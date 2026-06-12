import os
import paramiko

def download_file(ssh_client, remote_path, local_path):
    sftp = ssh_client.open_sftp()
    try:
        sftp.get(remote_path, local_path)
        print(f"Downloaded {remote_path} to {local_path}")
    except Exception as e:
        print(f"Failed to download {remote_path}: {e}")
    finally:
        sftp.close()

def main():
    # Replace with your robot's IP address and credentials
    robot_ip = "192.168.1.100"
    username = "user"
    password = "password"

    remote_file_path = "/path/to/remote/file"
    local_file_path = os.path.join(os.getcwd(), "downloaded_file")

    try:
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(robot_ip, username=username, password=password)

        download_file(ssh_client, remote_file_path, local_file_path)
    except Exception as e:
        print(f"Connection failed: {e}")
    finally:
        ssh_client.close()

if __name__ == "__main__":
    main()