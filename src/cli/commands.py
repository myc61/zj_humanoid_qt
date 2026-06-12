from argparse import ArgumentParser
from core.logger import Logger
from ros.version_check import check_version
from files.finder import find_file
from transfer.upload import upload_file
from transfer.download import download_file

logger = Logger()

def create_parser():
    parser = ArgumentParser(description="Humanoid Robot Delivery Toolchain CLI")
    subparsers = parser.add_subparsers(dest='command')

    version_parser = subparsers.add_parser('check_version', help='Check software version on the robot')
    version_parser.set_defaults(func=check_software_version)

    find_parser = subparsers.add_parser('find_file', help='Find a file on the robot')
    find_parser.add_argument('filename', type=str, help='Name of the file to find')
    find_parser.set_defaults(func=find_robot_file)

    upload_parser = subparsers.add_parser('upload', help='Upload a file to the robot')
    upload_parser.add_argument('local_path', type=str, help='Path to the local file')
    upload_parser.set_defaults(func=upload_robot_file)

    download_parser = subparsers.add_parser('download', help='Download a file from the robot')
    download_parser.add_argument('remote_path', type=str, help='Path to the file on the robot')
    download_parser.set_defaults(func=download_robot_file)

    return parser

def check_software_version(args):
    version_info = check_version()
    logger.info(f"Software version on the robot: {version_info}")

def find_robot_file(args):
    result = find_file(args.filename)
    if result:
        logger.info(f"File found: {result}")
    else:
        logger.warning("File not found.")

def upload_robot_file(args):
    success = upload_file(args.local_path)
    if success:
        logger.info("File uploaded successfully.")
    else:
        logger.error("File upload failed.")

def download_robot_file(args):
    success = download_file(args.remote_path)
    if success:
        logger.info("File downloaded successfully.")
    else:
        logger.error("File download failed.")

def main():
    parser = create_parser()
    args = parser.parse_args()
    if args.command:
        args.func(args)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()