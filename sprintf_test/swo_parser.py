import telnetlib
import sys
import time
import argparse

# Basic SWO ITM decoder
# Connects to OpenOCD's Tcl port to read ITM data
# Usage: python swo_parser.py [--dont-run]

DEFAULT_TCL_PORT = 6666
ITM_PORT = 0 # The stimulus port we're using for printf

def main():
    parser = argparse.ArgumentParser(description='SWO ITM Parser.')
    parser.add_argument('--tcl-port', type=int, default=DEFAULT_TCL_PORT,
                        help=f'OpenOCD Tcl server port (default: {DEFAULT_TCL_PORT})')
    parser.add_argument('--dont-run', action='store_true',
                        help='Do not send "reset run" command (useful when debugger is already running)')
    args = parser.parse_args()

    try:
        tn = telnetlib.Telnet("localhost", args.tcl_port)
        print(f"Connected to OpenOCD Tcl server on port {args.tcl_port}")

        # Optional: Clear any pending ITM trace
        tn.write(b"itm clear\n")
        tn.read_until(b">", timeout=1)

        # Enable ITM trace output to the Tcl server
        tn.write(b"itm port 0 on\n") # Enable port 0 explicitly (redundant with platformio.ini but safe)
        tn.read_until(b">", timeout=1)

        # Start collecting ITM data
        tn.write(b"itm_recv\n")
        tn.read_until(b">", timeout=1) # Read the initial prompt

        if not args.dont_run:
            # Optionally, reset and run the target.
            # USE WITH CAUTION if you are actively debugging, as it will reset your session!
            print("Sending 'reset run' command. Use --dont-run if debugger is active.")
            tn.write(b"reset run\n")
            tn.read_until(b">", timeout=1)
            time.sleep(0.1) # Give it a moment to start

        print("--- SWO Output ---")
        print("Waiting for ITM data... (Press Ctrl+C to stop)")

        while True:
            try:
                # Read all available data
                data = tn.read_very_eager()
                if data:
                    # ITM data often comes as "target.itm <port> <hex_data>"
                    # We just want the raw data from our port 0
                    lines = data.decode('utf-8').splitlines()
                    for line in lines:
                        if line.startswith(f"target.itm {ITM_PORT}"):
                            try:
                                # Extract hex string and convert to char
                                hex_str = line.split(' ')[2]
                                char_code = int(hex_str, 16)
                                sys.stdout.write(chr(char_code))
                                sys.stdout.flush()
                            except (IndexError, ValueError):
                                # Handle malformed lines gracefully
                                pass
                        elif line.strip() == ">":
                            # OpenOCD prompt, ignore or handle if needed
                            pass
                        elif line.strip() != "":
                            # Other OpenOCD messages, often harmless, can print for debugging
                            # print(f"[OpenOCD] {line.strip()}")
                            pass

                time.sleep(0.01) # Small delay to prevent busy-waiting
            except EOFError:
                print("\nConnection closed by OpenOCD.")
                break
            except KeyboardInterrupt:
                print("\nSWO Monitor stopped by user.")
                break

    except ConnectionRefusedError:
        print(f"Error: Could not connect to OpenOCD Tcl server on port {args.tcl_port}.")
        print("Please ensure OpenOCD is running (e.g., by starting a PlatformIO debug session).")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()