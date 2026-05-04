#!/bin/bash

# Fixed Bandwidth Limiter Script with proper filtering
INTERFACE="wlan0"
LIMIT="1mbit"
BURST="32k"

echo "Working with interface: $INTERFACE"

# Function to completely remove existing configuration
remove_existing_config() {
    echo "Removing existing configuration from $INTERFACE..."
    sudo tc qdisc del dev $INTERFACE root 2>/dev/null
    sudo tc qdisc del dev $INTERFACE ingress 2>/dev/null
    sleep 1
    echo "Existing configuration cleared"
}

# Function to setup bandwidth limiting with proper filtering
setup_bandwidth_limit() {
    echo "Setting up bandwidth limit of 1 Mbps on interface $INTERFACE"
    
    # Remove existing configuration first
    remove_existing_config
    
    # Wait for cleanup to complete
    sleep 2
    
    # METHOD: HTB with proper filters
    echo "Setting up HTB with filters..."
    
    # Step 1: Add root qdisc FIRST
    sudo tc qdisc add dev $INTERFACE root handle 1: htb default 30
    
    # Step 2: Add root class
    sudo tc class add dev $INTERFACE parent 1: classid 1:1 htb rate $LIMIT burst $BURST
    
    # Step 3: Add leaf class for all traffic
    sudo tc class add dev $INTERFACE parent 1:1 classid 1:30 htb rate $LIMIT burst $BURST
    
    # Step 4: Add SFQ for fairness (optional)
    sudo tc qdisc add dev $INTERFACE parent 1:30 handle 30: sfq perturb 10 2>/dev/null
    
    # Step 5: CORRECTED FILTERS - use proper u32 syntax
    # Filter for all IPv4 traffic (both directions)
    sudo tc filter add dev $INTERFACE protocol ip parent 1: prio 1 u32 \
        match ip protocol 0 0xff flowid 1:30
    
    # Alternative simpler method: use fw or basic filter if u32 is tricky
    # Or add a catch-all filter with basic match
    sudo tc filter add dev $INTERFACE parent 1: prio 2 basic match 'meta(priority eq 0)' flowid 1:30 2>/dev/null
    
    echo "✓ Bandwidth limit set to 1 Mbps on $INTERFACE"
    
    # Show the configuration
    echo "Current configuration:"
    echo ""
    echo "=== QDISCs ==="
    sudo tc -s qdisc show dev $INTERFACE
    echo ""
    echo "=== CLASSES ==="
    sudo tc -s class show dev $INTERFACE
    echo ""
    echo "=== FILTERS ==="
    sudo tc filter show dev $INTERFACE
}

# Alternative simpler method using TBF (Token Bucket Filter)
setup_simple_limit() {
    echo "Setting up simple TBF limit on $INTERFACE"
    remove_existing_config
    sleep 2
    
    # TBF is simpler and more reliable for simple rate limiting
    sudo tc qdisc add dev $INTERFACE root tbf rate $LIMIT burst $BURST latency 50ms
    
    echo "✓ Simple TBF limit set to 1 Mbps on $INTERFACE"
    sudo tc -s qdisc show dev $INTERFACE
}

# Function to remove bandwidth limiting
remove_bandwidth_limit() {
    echo "Removing bandwidth limits from interface $INTERFACE"
    remove_existing_config
    echo "✓ Bandwidth limits removed from $INTERFACE"
}

# Function to show current configuration
show_status() {
    echo "=== Current TC Configuration for $INTERFACE ==="
    echo ""
    echo "=== QDISCs ==="
    sudo tc -s qdisc show dev $INTERFACE
    echo ""
    echo "=== CLASSES ==="
    sudo tc -s class show dev $INTERFACE 2>/dev/null
    echo ""
    echo "=== FILTERS ==="
    sudo tc filter show dev $INTERFACE
}

# Function to monitor traffic in real-time
monitor_traffic() {
    echo "Monitoring traffic on $INTERFACE (Ctrl+C to stop)"
    echo ""
    while true; do
        clear
        date
        echo ""
        echo "=== Current Statistics ==="
        sudo tc -s class show dev $INTERFACE
        sleep 2
    done
}

# Main script
case "$1" in
    start)
        setup_bandwidth_limit
        ;;
    simple)
        setup_simple_limit
        ;;
    stop)
        remove_bandwidth_limit
        ;;
    status)
        show_status
        ;;
    monitor)
        monitor_traffic
        ;;
    *)
        echo "Usage: $0 {start|simple|stop|status|monitor}"
        echo "  start   - Setup HTB bandwidth limit with filters"
        echo "  simple  - Setup simple TBF limit (more reliable)"
        echo "  stop    - Remove bandwidth limits"
        echo "  status  - Show current configuration"
        echo "  monitor - Monitor traffic in real-time"
        exit 1
        ;;
esac