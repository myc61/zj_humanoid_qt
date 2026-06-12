import os
import stat
import shlex
import time
import threading
import paramiko


class InteractiveShellSession:
    def __init__(self, channel, on_output=None, on_exit=None):
        self._channel = channel
        self._on_output = on_output
        self._on_exit = on_exit
        self._stdout_chunks = []
        self._stderr_chunks = []
        self._pending = ""
        self._closed = threading.Event()
        self._reader = threading.Thread(target=self._pump_output, daemon=True)
        self._reader.start()

    def _emit_text(self, text: str):
        if not text:
            return
        self._pending += text.replace("\r", "\n")
        while "\n" in self._pending:
            line, self._pending = self._pending.split("\n", 1)
            if self._on_output:
                self._on_output(line)

    def _pump_output(self):
        exit_code = None
        try:
            while not self._closed.is_set():
                while self._channel.recv_ready():
                    data = self._channel.recv(4096)
                    if not data:
                        break
                    text = data.decode("utf-8", errors="ignore")
                    self._stdout_chunks.append(text)
                    self._emit_text(text)
                while self._channel.recv_stderr_ready():
                    data = self._channel.recv_stderr(4096)
                    if not data:
                        break
                    text = data.decode("utf-8", errors="ignore")
                    self._stderr_chunks.append(text)
                    self._emit_text(text)
                if self._channel.exit_status_ready() and not self._channel.recv_ready() and not self._channel.recv_stderr_ready():
                    exit_code = self._channel.recv_exit_status()
                    break
                time.sleep(0.05)
        finally:
            if self._pending and self._on_output:
                self._on_output(self._pending)
                self._pending = ""
            self._closed.set()
            if self._on_exit:
                self._on_exit(
                    "".join(self._stdout_chunks),
                    "".join(self._stderr_chunks),
                    exit_code,
                )

    def send(self, text: str = "", add_newline: bool = True):
        if self._closed.is_set() or self._channel.closed:
            raise RuntimeError("交互会话已关闭")
        payload = "" if text is None else str(text)
        if add_newline and not payload.endswith("\n"):
            payload += "\n"
        self._channel.send(payload)

    def interrupt(self):
        if self._closed.is_set() or self._channel.closed:
            return
        self._channel.send("\x03")

    def close(self):
        self._closed.set()
        try:
            self._channel.close()
        except Exception:
            pass

    def is_active(self) -> bool:
        return (not self._closed.is_set()) and (not self._channel.closed)


class SshSftpClient:
    def __init__(self):
        self.ssh = None
        self.sftp = None

    def connect(self, host: str, port: int, username: str, password: str):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(
            hostname=host,
            port=port,
            username=username,
            password=password,
            timeout=15,
            allow_agent=False,
            look_for_keys=False,
        )
        self.sftp = self.ssh.open_sftp()

    def connect_via_jump(self, jump_client: "SshSftpClient", host: str, port: int, username: str, password: str):
        if not jump_client or not jump_client.ssh:
            raise RuntimeError("跳板机 SSH 未连接")
        jump_transport = jump_client.ssh.get_transport()
        if not jump_transport or not jump_transport.is_active():
            raise RuntimeError("跳板机连接不可用")

        channel = jump_transport.open_channel("direct-tcpip", (host, int(port)), ("127.0.0.1", 0))

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(
            hostname=host,
            port=int(port),
            username=username,
            password=password,
            timeout=8,
            sock=channel,
            allow_agent=False,
            look_for_keys=False,
        )
        self.sftp = self.ssh.open_sftp()

    def close(self):
        if self.sftp:
            self.sftp.close()
            self.sftp = None
        if self.ssh:
            self.ssh.close()
            self.ssh = None

    def run(self, cmd: str):
        if not self.ssh:
            raise RuntimeError("SSH 未连接")
        _, stdout, stderr = self.ssh.exec_command(cmd)
        return stdout.read().decode("utf-8", errors="ignore"), stderr.read().decode("utf-8", errors="ignore")

    def run_interactive(self, cmd: str):
        if not self.ssh:
            raise RuntimeError("SSH 未连接")
        shell_cmd = f"bash -ic {shlex.quote(cmd)}"
        _, stdout, stderr = self.ssh.exec_command(shell_cmd, get_pty=True)
        channel = stdout.channel
        stdout_chunks = []
        stderr_chunks = []

        while True:
            while channel.recv_ready():
                data = channel.recv(4096)
                if not data:
                    break
                stdout_chunks.append(data.decode("utf-8", errors="ignore"))
            while channel.recv_stderr_ready():
                data = channel.recv_stderr(4096)
                if not data:
                    break
                stderr_chunks.append(data.decode("utf-8", errors="ignore"))
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            time.sleep(0.05)

        return "".join(stdout_chunks), "".join(stderr_chunks)

    def run_interactive_stream(self, cmd: str, on_output=None):
        if not self.ssh:
            raise RuntimeError("SSH 未连接")
        shell_cmd = f"bash -ic {shlex.quote(cmd)}"
        _, stdout, stderr = self.ssh.exec_command(shell_cmd, get_pty=True)
        channel = stdout.channel
        stdout_chunks = []
        stderr_chunks = []
        pending = ""

        def _push_text(text: str):
            nonlocal pending
            if not text:
                return
            pending += text.replace("\r", "\n")
            while "\n" in pending:
                line, pending = pending.split("\n", 1)
                if on_output:
                    on_output(line)

        while True:
            while channel.recv_ready():
                data = channel.recv(4096)
                if not data:
                    break
                text = data.decode("utf-8", errors="ignore")
                stdout_chunks.append(text)
                _push_text(text)
            while channel.recv_stderr_ready():
                data = channel.recv_stderr(4096)
                if not data:
                    break
                text = data.decode("utf-8", errors="ignore")
                stderr_chunks.append(text)
                _push_text(text)
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            time.sleep(0.05)

        if pending and on_output:
            on_output(pending)

        return "".join(stdout_chunks), "".join(stderr_chunks)

    def start_interactive_session(self, cmd: str, on_output=None, on_exit=None):
        if not self.ssh:
            raise RuntimeError("SSH 未连接")
        shell_cmd = f"bash -ic {shlex.quote(cmd)}"
        transport = self.ssh.get_transport()
        if not transport or not transport.is_active():
            raise RuntimeError("SSH 连接不可用")
        channel = transport.open_session()
        channel.get_pty()
        channel.exec_command(shell_cmd)
        return InteractiveShellSession(channel, on_output=on_output, on_exit=on_exit)

    def upload(self, local_path: str, remote_path: str):
        if not self.sftp:
            raise RuntimeError("SFTP 未连接")
        self.sftp.put(local_path, remote_path)

    def download(self, remote_path: str, local_path: str):
        if not self.sftp:
            raise RuntimeError("SFTP 未连接")
        parent = os.path.dirname(local_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self.sftp.get(remote_path, local_path)

    def _is_remote_dir(self, remote_path: str) -> bool:
        if not self.sftp:
            raise RuntimeError("SFTP 未连接")
        try:
            mode = self.sftp.stat(remote_path).st_mode
            return stat.S_ISDIR(mode)
        except Exception:
            return False

    def _mkdir_p(self, remote_dir: str):
        if not self.sftp:
            raise RuntimeError("SFTP 未连接")
        remote_dir = remote_dir.replace("\\", "/")
        parts = [p for p in remote_dir.split("/") if p]
        cur = "/" if remote_dir.startswith("/") else ""
        for p in parts:
            cur = f"{cur}/{p}" if cur else p
            try:
                self.sftp.stat(cur)
            except Exception:
                self.sftp.mkdir(cur)

    def upload_dir(self, local_dir: str, remote_dir: str):
        if not self.sftp:
            raise RuntimeError("SFTP 未连接")
        local_dir = os.path.abspath(local_dir)
        self._mkdir_p(remote_dir)

        for root, dirs, files in os.walk(local_dir):
            rel = os.path.relpath(root, local_dir)
            rel = "" if rel == "." else rel.replace("\\", "/")
            remote_root = remote_dir.rstrip("/") if rel == "" else f"{remote_dir.rstrip('/')}/{rel}"
            self._mkdir_p(remote_root)

            for d in dirs:
                self._mkdir_p(f"{remote_root}/{d}")

            for f in files:
                self.sftp.put(os.path.join(root, f), f"{remote_root}/{f}")

    def download_dir(self, remote_dir: str, local_dir: str):
        if not self.sftp:
            raise RuntimeError("SFTP 未连接")
        os.makedirs(local_dir, exist_ok=True)

        def _walk(rdir: str, ldir: str):
            os.makedirs(ldir, exist_ok=True)
            for entry in self.sftp.listdir_attr(rdir):
                rpath = f"{rdir.rstrip('/')}/{entry.filename}"
                lpath = os.path.join(ldir, entry.filename)
                if stat.S_ISDIR(entry.st_mode):
                    _walk(rpath, lpath)
                else:
                    self.sftp.get(rpath, lpath)

        _walk(remote_dir, local_dir)

    def export_logs(self, remote_log_dir: str, local_dir: str):
        os.makedirs(local_dir, exist_ok=True)
        remote_tar = "/tmp/robot_logs_export.tar.gz"
        self.run(f"tar -czf {remote_tar} -C {remote_log_dir} .")
        local_tar = os.path.join(local_dir, "robot_logs_export.tar.gz")
        self.download(remote_tar, local_tar)
        return local_tar