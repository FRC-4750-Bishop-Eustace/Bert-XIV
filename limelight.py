import ipaddress
import requests
import threading
import socket
import websocket
import json
import ifaddr

def broadcast_message(message, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(message.encode(), ('255.255.255.255', port))

def broadcast_on_all_interfaces(message, port, debug):
    # First, gather all network information
    networks = []
    for adapter in ifaddr.get_adapters():
        for ip in adapter.ips:
            if isinstance(ip.ip, str):  # Checks for IPv4 based on whether it's a string
                net = ipaddress.ip_network(f"{ip.ip}/{ip.network_prefix}", strict=False)
                networks.append((adapter.name, ip.ip, net.broadcast_address))
            else:
                continue  # Skip non-IPv4 addresses

    # Print all discovered network interfaces
    for name, ip, broadcast in networks:
        if debug:
            print(f"Adapter: {name}, IP: {ip}, Broadcast: {broadcast}")

    # Now, broadcast the message on each network
    for _, _, broadcast in networks:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.sendto(message.encode(), (str(broadcast), port))
        except Exception as e:
            print(f"Failed to broadcast on {broadcast}: {e}")

def listen_for_responses(port,timeout=1):
    discovered_devices = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.bind(("", port))
        sock.settimeout(timeout)
        try:
            while True:
                data, addr = sock.recvfrom(1024)
                discovered_devices.append(addr[0])
                #print(f"Received data from {addr}: {data.decode()}")
        except socket.timeout:
            pass
    return discovered_devices

def discover_limelights(broadcast_port=5809, listen_port=5809, timeout=2, debug=False):
    broadcast_on_all_interfaces("LLPhoneHome",broadcast_port,debug)
    return listen_for_responses(listen_port,timeout)

class Limelight:
    def __init__(self, address):
        self.base_url = f"http://{address}:5807"
        self.ws_url = f"ws://{address}:5806"
        self.latest_results = None
        self.ws = None
        self.ws_thread = None

    def get_results(self):
        return requests.get(f"{self.base_url}/results").json()
    
    def get_status(self):
        response = requests.get(f"{self.base_url}/status")
        if response.ok:
            return response.json()
        else:
            return None
    
    # Get hardware characteristics
    def hw_report(self):
        return requests.get(f"{self.base_url}/hwreport").json()

    # Force the camera to reload the current pipeline and all pipeline resources
    def reload_pipeline(self):
        return requests.post(f"{self.base_url}/reload-pipeline")

    # Get default pipeline
    def get_pipeline_default(self):
        response = requests.get(f"{self.base_url}/pipeline-default")
        if response.ok:
            return response.json()
        else:
            return None

    # Get pipeline[n] from the camera
    def get_pipeline_atindex(self, index):
        params = {'index': index}
        response = requests.get(f"{self.base_url}/pipeline-atindex", params=params)
        if response.ok:
            return response.json()
        else:
            return None

    # Switch to a different pipeline
    def pipeline_switch(self, index):
        params = {'index': index}
        return requests.post(f"{self.base_url}/pipeline-switch", params=params)

    # List of snapscriptpro names
    def get_snapscript_names(self):
        return requests.get(f"{self.base_url}/getsnapscriptnames").json()

    def capture_snapshot(self, snapname=""):
        params = {'snapname': snapname}
        return requests.post(f"{self.base_url}/capture-snapshot", params=params)

    def upload_snapshot(self, snapname, image_path):
        params = {'snapname': snapname}
        with open(image_path, 'rb') as image_file:
            files = {'file': image_file}
            return requests.post(f"{self.base_url}/upload-snapshot", params=params, files=files)

    def snapshot_manifest(self):
        return requests.get(f"{self.base_url}/snapshotmanifest").json()

    def delete_snapshots(self):
        return requests.delete(f"{self.base_url}/delete-snapshots")


    def delete_snapshot(self, snapname):
        params = {'snapname': snapname}
        return requests.delete(f"{self.base_url}/delete-snapshot",params=params)

    # Accepts Json with one or more settings updates. Set "flush" to 1 to save these settings to disk
    def update_pipeline(self, profile_json, flush=None):
        headers = {'Content-Type': 'application/json'}
        params = {}
        if flush is not None:
            params['flush'] = flush
        print(profile_json)
        response = requests.post(f"{self.base_url}/update-pipeline", headers=headers, params=params, data=profile_json)
        if response.status_code == 400:
            # Check if the response is JSON-formatted
            try:
                error_details = response.json()  # Parse JSON to get error message
                print("Error:", error_details)
            except ValueError:
                # If response is not JSON, print raw text
                print("Error:", response.text)
        return response


    def update_python_inputs(self, inputs):
        data = json.dumps(inputs)
        headers = {'Content-Type': 'application/json'}
        response = requests.post(f"{self.base_url}/update-pythoninputs", headers=headers, data=data)
        return response

    def update_robot_orientation(self, orientation_data):
        data = json.dumps(orientation_data) 
        headers = {'Content-Type': 'application/json'}
        response = requests.post(f"{self.base_url}/update-robotorientation", headers=headers, data=data)
        return response


    def upload_pipeline(self, profile_json, index=None):
        headers = {'Content-Type': 'application/json'}
        params = {}
        if index is not None:
            params['index'] = index

        response = requests.post(f"{self.base_url}/upload-pipeline", headers=headers, params=params, data=profile_json)
        return response

    def upload_fieldmap(self, fieldmap_json, index=None):
        headers = {'Content-Type': 'application/json'}
        params = {}
        if index is not None:
            params['index'] = index

        response = requests.post(f"{self.base_url}/upload-fieldmap", headers=headers, params=params, data=fieldmap_json)
        return response

    def upload_python(self, pythonstring, index=None):
        headers = {'Content-Type': 'text/plain'}
        params = {}
        if index is not None:
            params['index'] = index

        response = requests.post(f"{self.base_url}/upload-python", headers=headers, params=params, data=pythonstring)
        return response


    def upload_neural_network(self, nn_type, file_path, index=None):
        params = {'type': nn_type}
        if index is not None:
            params['index'] = index

        with open(file_path, 'rb') as nn_file:
            headers = {'Content-Type': 'application/octet-stream'}
            file_contents = nn_file.read()
            return requests.post(f"{self.base_url}/upload-nn", params=params, headers=headers, data=file_contents)

    def upload_neural_network_labels(self, nn_type, file_path, index=None):
        params = {'type': nn_type}
        if index is not None:
            params['index'] = index

        with open(file_path, 'rb') as nn_file:
            headers = {'Content-Type': 'text/plain'}
            file_contents = nn_file.read()
            return requests.post(f"{self.base_url}/upload-nnlabels", params=params, headers=headers, data=file_contents)



    def cal_default(self):
        return requests.get(f"{self.base_url}/cal-default").json()

    def cal_file(self):
        return requests.get(f"{self.base_url}/cal-file").json()

    def cal_eeprom(self):
        return requests.get(f"{self.base_url}/cal-eeprom").json()

    def cal_latest(self):
        return requests.get(f"{self.base_url}/cal-latest").json()

    def update_cal_eeprom(self, cal_data):
        return requests.post(f"{self.base_url}/cal-eeprom", data=cal_data)

    def update_cal_file(self, cal_data):
        return requests.post(f"{self.base_url}/cal-file", data=cal_data)

    def delete_cal_latest(self):
        return requests.delete(f"{self.base_url}/cal-latest")

    def delete_cal_eeprom(self):
        return requests.delete(f"{self.base_url}/cal-eeprom")

    def delete_cal_file(self):
        return requests.delete(f"{self.base_url}/cal-file")

   

    def get_name(self):
        status = self.get_status()
        if status:
            return status.get('name', None)
        return None

    def get_temp(self):
        status = self.get_status()
        if status:
            return status.get('temp', None)
        return None

    def get_fps(self):
        status = self.get_status()
        if status:
            return status.get('fps', None)
        return None

    def enable_websocket(self):
        def on_message(ws, message):
            self.latest_results = json.loads(message)
        def on_error(ws, error):
            print(f"WebSocket error: {error}")
        def on_close(ws):
            print("WebSocket closed")
        def run(*args):
            self.ws = websocket.WebSocketApp(self.ws_url,
                                             on_message=on_message,
                                             on_error=on_error,
                                             on_close=on_close)
            self.ws.run_forever()
        self.ws_thread = threading.Thread(target=run)
        self.ws_thread.start()

    def disable_websocket(self):
        if self.ws:
            self.ws.close()
            self.ws_thread.join()
            print("LL websocket disabled.")

    def get_latest_results(self):
        return self.latest_results
