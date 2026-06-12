import json
import shlex
import threading
import time
from typing import Any, Callable, Dict, Optional
from urllib.parse import urlparse

import roslibpy
import yaml


class BridgeClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 9090):
        self.host = host
        self.port = int(port)
        self.ros = roslibpy.Ros(host=self.host, port=self.port)
        self._conn_lock = threading.RLock()
        self._subscriptions: Dict[str, roslibpy.Topic] = {}
        self._rpc_lock = threading.RLock()
        self._service_type_cache: Dict[str, str] = {}
        self._topic_type_cache: Dict[str, str] = {}

    def connect(self, timeout: float = 6.0):
        with self._conn_lock:
            if self.ros.is_connected:
                return True

            def _wait_connected() -> bool:
                deadline = time.time() + float(timeout)
                while time.time() < deadline:
                    if self.ros.is_connected:
                        return True
                    time.sleep(0.1)
                return False

            # 先尝试复用当前对象，失败后再重建对象尝试
            try:
                self.ros.run()
                if _wait_connected():
                    return True
            except Exception:
                pass

            try:
                self.ros.close()
            except Exception:
                pass

            self.ros = roslibpy.Ros(host=self.host, port=self.port)
            try:
                self.ros.run()
            except Exception:
                return False
            return _wait_connected()

    def check_connection(self):
        return self.ros.is_connected

    def disconnect(self):
        with self._conn_lock:
            for _, sub in list(self._subscriptions.items()):
                try:
                    sub.unsubscribe()
                except Exception:
                    pass
            self._subscriptions.clear()
            self._service_type_cache.clear()
            self._topic_type_cache.clear()

            # 不调用 terminate，避免底层事件循环被停止后无法重连
            try:
                self.ros.close()
            except Exception:
                pass

            deadline = time.time() + 2.0
            while time.time() < deadline:
                if not self.ros.is_connected:
                    break
                time.sleep(0.05)

            # 主动重建对象，避免复用已关闭对象导致后续 run 行为不稳定
            self.ros = roslibpy.Ros(host=self.host, port=self.port)

    def _normalize_response(self, value: Any):
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        if isinstance(value, dict):
            return {str(k): self._normalize_response(v) for k, v in value.items()}
        if isinstance(value, (list, tuple, set)):
            return [self._normalize_response(v) for v in value]

        data = getattr(value, "data", None)
        if isinstance(data, dict):
            return self._normalize_response(data)

        values = getattr(value, "values", None)
        if isinstance(values, dict):
            return self._normalize_response(values)

        try:
            attrs = vars(value)
        except Exception:
            attrs = None
        if isinstance(attrs, dict) and attrs:
            public_attrs = {k: v for k, v in attrs.items() if not str(k).startswith("_")}
            if public_attrs:
                return self._normalize_response(public_attrs)

        return str(value)

    def _call_service_sync(self, service_name: str, service_type: str, request: Optional[Dict[str, Any]] = None, timeout: float = 20.0):
        with self._rpc_lock:
            done = threading.Event()
            box: Dict[str, Any] = {"resp": None, "err": None}
            service = roslibpy.Service(self.ros, service_name, service_type)

            def cb(resp: Dict[str, Any]):
                box["resp"] = resp
                done.set()

            def eb(err: Any):
                box["err"] = err
                done.set()

            service.call(roslibpy.ServiceRequest(request or {}), callback=cb, errback=eb)
            done.wait(timeout=timeout)

            if box["err"] is not None:
                raise RuntimeError(str(box["err"]))
            if box["resp"] is None:
                raise TimeoutError(f"Service call timeout: {service_name}")
            return self._normalize_response(box["resp"])

    def _get_service_type(self, service_name: str) -> str:
        cached = self._service_type_cache.get(service_name)
        if cached:
            return cached
        resp = self._call_service_sync("/rosapi/service_type", "rosapi/ServiceType", {"service": service_name}, timeout=3.0)
        st = resp.get("type", "")
        if st:
            self._service_type_cache[service_name] = st
        return st

    def _get_topic_type(self, topic_name: str) -> str:
        cached = self._topic_type_cache.get(topic_name)
        if cached:
            return cached
        resp = self._call_service_sync("/rosapi/topic_type", "rosapi/TopicType", {"topic": topic_name}, timeout=3.0)
        tt = resp.get("type", "")
        if tt:
            self._topic_type_cache[topic_name] = tt
        return tt

    def request_service(self, service_name: str, request: Optional[Dict[str, Any]] = None, service_type: Optional[str] = None, timeout: float = 20.0):
        if not service_type:
            service_type = self._get_service_type(service_name)
        if not service_type:
            raise RuntimeError(f"Cannot resolve service type: {service_name}")
        return self._call_service_sync(service_name, service_type, request or {}, timeout=float(timeout))

    def list_nodes(self):
        resp = self._call_service_sync("/rosapi/nodes", "rosapi/Nodes", {})
        return resp.get("nodes", [])

    def list_topics(self):
        resp = self._call_service_sync("/rosapi/topics", "rosapi/Topics", {})
        return resp.get("topics", [])

    def list_services(self):
        resp = self._call_service_sync("/rosapi/services", "rosapi/Services", {})
        return resp.get("services", [])

    def subscribe(self, topic_name: str, callback: Callable[[Dict[str, Any]], None], topic_type: Optional[str] = None):
        if not topic_type:
            topic_type = self._get_topic_type(topic_name)
        if not topic_type:
            raise RuntimeError(f"无法获取话题类型: {topic_name}")
        topic = roslibpy.Topic(self.ros, topic_name, topic_type)
        topic.subscribe(callback)
        self._subscriptions[topic_name] = topic
        return topic_type

    def unsubscribe(self, topic_name: str):
        sub = self._subscriptions.get(topic_name)
        if sub:
            sub.unsubscribe()
            self._subscriptions.pop(topic_name, None)

    def _parse_rosservice_request(self, req_tokens):
        if not req_tokens:
            return {}

        req_text = " ".join(req_tokens).strip()
        if not req_text:
            return {}

        # 1) JSON
        try:
            parsed = json.loads(req_text)
            if parsed is None:
                return {}
            if not isinstance(parsed, dict):
                raise ValueError("请求体必须是对象")
            return parsed
        except Exception:
            pass

        # 2) ROS CLI 常见写法: key:=value
        cli_style = {}
        cli_ok = True
        for t in req_tokens:
            if ":=" not in t:
                cli_ok = False
                break
            k, v = t.split(":=", 1)
            k = k.strip()
            if not k:
                cli_ok = False
                break
            # 尝试把值解析为数字/布尔/列表等，否则保留字符串
            try:
                vv = yaml.safe_load(v)
            except Exception:
                vv = v
            cli_style[k] = vv
        if cli_ok:
            return cli_style

        # 3) YAML: 支持 "{arm_type: 1}" 或 "arm_type: 1"
        try:
            parsed = yaml.safe_load(req_text)
            if parsed is None:
                return {}
            if not isinstance(parsed, dict):
                raise ValueError("请求体必须是对象")
            return parsed
        except Exception as e:
            raise ValueError(f"无法解析 service 请求参数: {req_text} ({e})")

    def execute_command(self, command: str, on_output: Callable[[str], None]):
        tokens = shlex.split(command.strip())
        if len(tokens) < 2:
            on_output("命令格式错误")
            return None

        if tokens[0] == "rosservice" and tokens[1] == "call":
            if len(tokens) < 3:
                on_output("用法: rosservice call /service_name '{\"key\":\"value\"}'")
                return None
            service_name = tokens[2]
            req = self._parse_rosservice_request(tokens[3:])
            resp = self.request_service(service_name, req)
            on_output(json.dumps(resp, ensure_ascii=False, indent=2))
            return {"kind": "service_call", "service": service_name}

        if tokens[0] == "rostopic" and tokens[1] == "echo":
            if len(tokens) < 3:
                on_output("用法: rostopic echo /topic_name [--once]")
                return None
            topic_name = tokens[2]
            once = "--once" in tokens

            got_first = {"done": False}

            def _topic_cb(msg: Dict[str, Any]):
                on_output(json.dumps(msg, ensure_ascii=False))
                if once and not got_first["done"]:
                    got_first["done"] = True
                    try:
                        self.unsubscribe(topic_name)
                    except Exception:
                        pass

            topic_type = self.subscribe(topic_name, _topic_cb)
            on_output(f"已订阅: {topic_name} ({topic_type})")
            if once:
                return {"kind": "topic_echo_once", "topic": topic_name, "topic_type": topic_type}
            return {"kind": "topic_echo", "topic": topic_name, "topic_type": topic_type}

        on_output(f"暂不支持命令: {command}")
        return None

    def find_file(self, file_name: str, find_service: str = "/find_file"):
        return self.request_service(find_service, {"file_name": file_name})

    def transfer_file(self, source_path, destination_path):
        raise NotImplementedError("请使用 paramiko 实现 SFTP 传输。")


class ROSConnection(BridgeClient):
    """兼容旧调用: ROSConnection(master_uri)"""

    def __init__(self, master_uri: str):
        host, port = self._parse_master_uri(master_uri)
        super().__init__(host=host, port=port)
        self.master_uri = master_uri

    @staticmethod
    def _parse_master_uri(master_uri: str):
        if master_uri.startswith("ws://") or master_uri.startswith("wss://"):
            p = urlparse(master_uri)
            return p.hostname or "127.0.0.1", p.port or 9090
        if master_uri.startswith("http://") or master_uri.startswith("https://"):
            p = urlparse(master_uri)
            return p.hostname or "127.0.0.1", 9090
        if ":" in master_uri:
            host, port = master_uri.split(":", 1)
            return host.strip(), int(port.strip())
        return master_uri.strip(), 9090