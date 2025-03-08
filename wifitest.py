import subprocess

def get_signal_strength_level(signal_strength):
    try:
        # Convert signal strength to integer
        signal_db = int(signal_strength)
        
        # Determine the level based on signal strength
        if signal_db >= -50:
            return 4  # Excellent
        elif -60 <= signal_db < -50:
            return 3  # Good
        elif -70 <= signal_db < -60:
            return 2  # Fair
        elif -80 <= signal_db < -70:
            return 1  # Poor
        else:
            return 0  # Very Poor or No Signal
    except ValueError:
        return 0  # Handle case where signal strength is not a valid number

def get_wifi_status():
    try:
        # Get SSID
        result = subprocess.run(['iwgetid'], capture_output=True, text=True)
        if result.returncode == 0:
            ssid = result.stdout.strip()
        else:
            ssid = "Not connected"

        # Get Signal Strength
        result = subprocess.run(['iwconfig'], capture_output=True, text=True)
        if result.returncode == 0:
            output = result.stdout
            signal_strength = "Unknown"
            for line in output.split('\n'):
                if 'Signal level' in line:
                    # Extract signal level from the line
                    signal_strength = line.split('Signal level=')[1].split(' ')[0]
                    break
        else:
            signal_strength = "Unknown"

        # Determine signal level
        level = get_signal_strength_level(signal_strength)

        # Print results
        print(f"Wi-Fi Status:")
        print(f"SSID: {ssid}")
        print(f"Signal Strength: {signal_strength} dBm")
        print(f"Signal Level: {level}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    get_wifi_status()
