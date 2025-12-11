import sys

## If you want to use the local version of the library, you can uncomment the following lines.
# PROJECT_PATH = '<your project path>'
# sys.path.insert(1, f'{PROJECT_PATH}')
# sys.path.insert(
#     1,
#     f'{PROJECT_PATH}/hex_device/generated')

from hex_device.common_utils import is_valid_ws_url, InvalidWSURLException

def test_valid_urls():
    """Test valid URLs (IPv4, domain, and IPv6)"""
    print("=== Testing valid URLs ===")
    try:
        # IPv4 and domain tests
        url, is_ipv6 = is_valid_ws_url('ws://localhost')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://localhost:8439
        url, is_ipv6 = is_valid_ws_url('wss://example.com')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # wss://example.com:8439
        url, is_ipv6 = is_valid_ws_url('ws://192.168.1.1:8080')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://192.168.1.1:8080
        
        # IPv6 tests - must include zone identifier (%3 or %eth0 format)
        url, is_ipv6 = is_valid_ws_url('ws://[::1%3]')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[::1%3]:8439 (localhost with numeric zone)
        url, is_ipv6 = is_valid_ws_url('ws://[::1%eth0]')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[::1%eth0]:8439 (localhost with interface name)
        url, is_ipv6 = is_valid_ws_url('ws://[2001:db8::1%3]:8080')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[2001:db8::1%3]:8080
        url, is_ipv6 = is_valid_ws_url('wss://[2001:0db8:85a3:0000:0000:8a2e:0370:7334%eth0]')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # wss://[2001:0db8:85a3:0000:0000:8a2e:0370:7334%eth0]:8439
        url, is_ipv6 = is_valid_ws_url('ws://[2001:db8::8a2e:370:7334%3]:9000')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[2001:db8::8a2e:370:7334%3]:9000
        url, is_ipv6 = is_valid_ws_url('wss://[::ffff:192.0.2.1%eth0]:443')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # wss://[::ffff:192.0.2.1%eth0]:443 (IPv4-mapped)
        url, is_ipv6 = is_valid_ws_url('ws://[2001:0db8:0000:0000:0000:0000:0000:0001%3]')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[2001:0db8:0000:0000:0000:0000:0000:0001%3]:8439 (full format)
        url, is_ipv6 = is_valid_ws_url('ws://[fe80::1%wlan0]')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[fe80::1%wlan0]:8439 (link-local with interface name)
        url, is_ipv6 = is_valid_ws_url('ws://[fe80::1%5]')
        print(f"✓ {url} (IPv6: {is_ipv6})")  # ws://[fe80::1%5]:8439 (link-local with numeric zone)
    except InvalidWSURLException as e:
        print(f"✗ Unexpected error: {e}")

def test_invalid_urls():
    """Test invalid URLs"""
    print("\n=== Testing invalid URLs ===")
    
    invalid_cases = [
        ("ws://192.168.1.1:70000", "Port out of range"),
        ("https://example.com", "Wrong protocol"),
        ("ws://example.com:abc", "Invalid port format"),
        ("ws://[::1", "IPv6 missing closing bracket"),
        ("ws://::1]", "IPv6 missing opening bracket"),
        ("ws://::1", "IPv6 without brackets"),
        ("ws://[invalid-ipv6]", "Invalid IPv6 address format"),
        ("ws://[gggg::1]", "Invalid IPv6 hex characters"),
        ("ws://[2001:db8::1:2:3:4:5:6:7]", "Invalid IPv6 (too many segments)"),
        ("ws://[::1]:99999", "IPv6 with invalid port (out of range)"),
        # IPv6 without zone identifier (now required)
        ("ws://[::1]", "IPv6 without zone identifier"),
        ("ws://[2001:db8::1]", "IPv6 without zone identifier"),
        ("ws://[fe80::1]", "IPv6 without zone identifier"),
        # Invalid zone identifier formats
        ("ws://[::1%]", "IPv6 with empty zone identifier"),
        ("ws://[::1%-]", "IPv6 with invalid zone identifier (contains dash)"),
        ("ws://[::1%eth-0]", "IPv6 with invalid zone identifier (contains dash)"),
        ("ws://[::1%eth.0]", "IPv6 with invalid zone identifier (contains dot)"),
        ("ws://[::1%eth 0]", "IPv6 with invalid zone identifier (contains space)"),
    ]
    
    for url, reason in invalid_cases:
        try:
            result_url, is_ipv6 = is_valid_ws_url(url)
            print(f"✗ Should fail but passed: {url} ({reason}) -> {result_url} (IPv6: {is_ipv6})")
        except InvalidWSURLException as e:
            print(f"✓ Correctly rejected: {url} ({reason})")
            print(f"  Error: {e}")

def main():
    test_valid_urls()
    test_invalid_urls()
    print("\n=== All tests completed ===")

if __name__ == "__main__":
    main()
