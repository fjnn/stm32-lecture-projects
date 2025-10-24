import time
import json
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer

# --- Global State and Configuration ---
HOST_NAME = '0.0.0.0'
PORT_NUMBER = 8080

# Global variable to hold the latest sensor reading
LATEST_DATA = {
    "timestamp": time.time(),
    "value": 0.0,
    "unit": "DEBUG",
}

# Minimal HTML Content for quick viewing and polling
SIMPLE_HTML_CONTENT = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>STM32 Debug Data</title>
</head>
<body>
    <h1>STM32 Debug Server</h1>
    <h2>Latest Received Data:</h2>
    <pre id="data">Loading...</pre>
    <p><b>To Send Data (STM32):</b> HTTP POST raw value to <code>http://[SERVER_IP]:{PORT_NUMBER}/update</code></p>
    
    <script>
        // Simple polling script
        function updateData() {
            fetch('/data')
                .then(res => res.json())
                .then(data => {
                    document.getElementById('data').textContent = JSON.stringify(data, null, 2);
                })
                .catch(err => {
                    document.getElementById('data').textContent = 'ERROR: Could not connect to server.';
                    console.error('Fetch error:', err);
                });
        }
        updateData();
        setInterval(updateData, 1000); // Poll every second
    </script>
</body>
</html>
""".replace("{PORT_NUMBER}", str(PORT_NUMBER))


# --- Custom HTTP Request Handler ---

class SimpleDataServerHandler(BaseHTTPRequestHandler):
    
    # Minimal logging override
    def log_message(self, format, *args):
        if self.path != '/data' or self.command != 'GET':
            super().log_message(format, *args)

    def do_GET(self):
        """Handle browser requests to view the page and fetch data."""
        global LATEST_DATA
        
        if self.path == '/':
            # Serve the main HTML page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(bytes(SIMPLE_HTML_CONTENT, "utf-8"))
            
        elif self.path == '/data':
            # Serve the latest data as JSON for the JavaScript client
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            json_response = json.dumps(LATEST_DATA)
            self.wfile.write(bytes(json_response, "utf-8"))
            
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        """Handle data transmission (HTTP POST) from the STM32 client."""
        global LATEST_DATA

        if self.path == '/update':
            # Read data size and body with minimal checks
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            
            try:
                # Assuming raw text data, strip and convert to float
                sensor_value_str = post_data.decode('utf-8').strip()
                new_value = float(sensor_value_str)
                
                # Update the global state
                LATEST_DATA["timestamp"] = time.time()
                LATEST_DATA["value"] = new_value
                
                print(f"[{time.strftime('%H:%M:%S', time.localtime())}] POST Received: {new_value}")
                
                # Success response
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(bytes("OK", "utf-8"))
                
            except Exception:
                # Simple bad request response if data conversion fails
                self.send_response(400)
                self.end_headers()
                self.wfile.write(bytes("Bad Data Format", "utf-8"))

        else:
            self.send_response(404)
            self.end_headers()

# --- Server Startup ---

def run_server():
    """Initializes and runs the custom HTTP server."""
    server_address = (HOST_NAME, PORT_NUMBER)
    httpd = HTTPServer(server_address, SimpleDataServerHandler)
    
    # Try to determine and print the local IP address for the MCU
    local_ip = '127.0.0.1'
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        pass

    print(f"{time.asctime()} | DEBUG Server Starting...")
    print(f"--------------------------------------------------")
    print(f"View Data (GET):   http://{local_ip}:{PORT_NUMBER}/")
    print(f"Send Data (POST):  http://{local_ip}:{PORT_NUMBER}/update")
    print(f"--------------------------------------------------")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print(f"\n{time.asctime()} | Server Stopped.")
        httpd.server_close()


if __name__ == '__main__':
    run_server()
