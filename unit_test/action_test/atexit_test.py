import atexit
import sys

def restart_program():
    python = sys.executable
    args = sys.argv
    args.insert(0, sys.executable)
    sys.exit(0)

def main():
    try:
        print(1)
        print(1/0)
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
        print("Restarting the program...")
        restart_program()

if __name__ == "__main__":
    atexit.register(restart_program)
    main()
