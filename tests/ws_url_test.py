from hex_device import is_valid_ws_url, InvalidWSURLException

def main():
    try:
        print(is_valid_ws_url("ws://localhost"))  # ws://localhost:8080
        print(is_valid_ws_url("wss://example.com"))  # wss://example.com:8080
        print(is_valid_ws_url("ws://192.168.1.1:70000"))  # panic
        print(is_valid_ws_url("https://example.com"))  # panic
        print(is_valid_ws_url("ws://example.com:abc"))  # panic
    except InvalidWSURLException as e:
        print(e)

if __name__ == "__main__":
    main()
