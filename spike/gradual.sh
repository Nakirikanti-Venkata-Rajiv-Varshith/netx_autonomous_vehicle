#!/bin/bash

# Bandwidth Limiter Script with Initial Zero Period
INTERFACE="wlan0"
ORIGINAL_LIMIT="30mbit"  # Target limit (30 Mbps)
BURST="32k"
STEPS=6  # Number of steps after zero period
STEP_DURATION=40  # Seconds between steps
ZERO_DURATION=40  # Initial zero bandwidth period

echo "Working with interface: $INTERFACE"
echo "Target limit: $ORIGINAL_LIMIT (30 Mbps)"
echo "Initial zero period: $ZERO_DURATION seconds"
echo "Steps after zero: $STEPS steps of $STEP_DURATION seconds each"
echo "Step increments: 5 Mbps every $STEP_DURATION seconds"

# Function to convert human-readable rate to kbps
convert_to_kbps() {
    local rate=$1
    case $rate in
        *kbit) echo ${rate%kbit} ;;
        *mbit) echo $((${rate%mbit} * 1024)) ;;
        *kbps) echo ${rate%kbps} ;;
        *mbps) echo $((${rate%mbps} * 1024)) ;;
        *) echo 0 ;;
    esac
}

# Function to convert kbps to human-readable format
convert_from_kbps() {
    local kbps=$1
    if [ $kbps -ge 1024 ]; then
        echo "$((kbps / 1024))mbit"
    else
        echo "${kbps}kbit"
    fi
}

# Function to completely remove existing configuration
remove_existing_config() {
    echo "Removing existing configuration from $INTERFACE..."
    sudo tc qdisc del dev $INTERFACE root 2>/dev/null
    sudo tc qdisc del dev $INTERFACE ingress 2>/dev/null
    sleep 1
    echo "Existing configuration cleared"
}

# Function to apply bandwidth limit
apply_limit() {
    local limit=$1
    local step_num=$2
    local limit_mbps=$3
    local phase=$4  # "zero" or "step"
    
    echo ""
    if [ "$phase" = "zero" ]; then
        echo "=== ZERO BANDWIDTH PHASE ==="
    else
        echo "=== Applying Step $step_num/$STEPS: $limit_mbps Mbps ($limit) ==="
    fi
    
    # Remove existing config for this step
    sudo tc qdisc del dev $INTERFACE root 2>/dev/null
    
    # Apply new limit
    sudo tc qdisc add dev $INTERFACE root handle 1: htb default 30
    sudo tc class add dev $INTERFACE parent 1: classid 1:1 htb rate $limit burst $BURST
    sudo tc class add dev $INTERFACE parent 1:1 classid 1:30 htb rate $limit burst $BURST
    sudo tc filter add dev $INTERFACE protocol ip parent 1: prio 1 u32 match ip protocol 0 0xff flowid 1:30
    
    if [ "$phase" = "zero" ]; then
        echo "✓ Applied ZERO bandwidth limit"
    else
        echo "✓ Applied limit: $limit_mbps Mbps ($limit)"
    fi
    
    # Show current rate
    CURRENT_RATE=$(tc class show dev $INTERFACE 2>/dev/null | grep "class htb" | head -1 | grep -o "rate [^ ]*" | cut -d' ' -f2)
    echo "  Current rate: $CURRENT_RATE"
}

# Function to run zero bandwidth period
run_zero_period() {
    echo ""
    echo "========================================"
    echo "PHASE 1: INITIAL ZERO BANDWIDTH PERIOD"
    echo "========================================"
    echo "Setting bandwidth to 0 for $ZERO_DURATION seconds"
    
    # Apply zero limit (using a very small rate since 0 might not work)
    apply_limit "1kbit" 0 0 "zero"
    
    echo "----------------------------------------"
    echo "Zero bandwidth period started. Countdown:"
    
    # Countdown for zero period
    for ((i=$ZERO_DURATION; i>0; i--)); do
        # Calculate progress percentage
        PROGRESS=$(( (ZERO_DURATION - i) * 100 / ZERO_DURATION ))
        
        # Create progress bar (40 chars wide)
        BAR="["
        for ((j=0; j<40; j++)); do
            if [ $j -lt $((PROGRESS * 40 / 100)) ]; then
                BAR="${BAR}█"
            else
                BAR="${BAR}░"
            fi
        done
        BAR="${BAR}]"
        
        # Get current stats if available
        STATS=$(tc -s class show dev $INTERFACE 2>/dev/null | grep -A 2 "class htb 1:30" | tail -1 | sed 's/^[[:space:]]*//')
        
        echo -ne "\r${BAR} ${PROGRESS}% | Time left: ${i}s | Stats: ${STATS:0:40}..."
        sleep 1
    done
    echo ""  # New line
    
    echo "✓ Zero bandwidth period complete!"
}

# Function to gradually increase bandwidth after zero period
gradual_increase() {
    echo ""
    echo "========================================"
    echo "PHASE 2: GRADUAL BANDWIDTH INCREASE"
    echo "========================================"
    echo "Increasing from 0 to 30 Mbps in $STEPS steps"
    echo "Each step: +5 Mbps every $STEP_DURATION seconds"
    
    # Calculate step sizes for 0-30 Mbps in 6 steps
    # Step 1: 5 Mbps
    # Step 2: 10 Mbps
    # Step 3: 15 Mbps
    # Step 4: 20 Mbps
    # Step 5: 25 Mbps
    # Step 6: 30 Mbps
    
    for ((step=1; step<=STEPS; step++)); do
        # Calculate current limit in Mbps (5, 10, 15, 20, 25, 30)
        CURRENT_MBPS=$((step * 5))
        CURRENT_LIMIT="${CURRENT_MBPS}mbit"
        
        apply_limit $CURRENT_LIMIT $step $CURRENT_MBPS "step"
        
        if [ $step -lt $STEPS ]; then
            NEXT_MBPS=$(( (step + 1) * 5 ))
            echo "----------------------------------------"
            echo "Next step: $NEXT_MBPS Mbps in $STEP_DURATION seconds"
            
            # Countdown timer with visual progress
            for ((i=$STEP_DURATION; i>0; i--)); do
                # Calculate progress percentage for this step
                STEP_PROGRESS=$(( (STEP_DURATION - i) * 100 / STEP_DURATION ))
                
                # Create progress bar (40 chars wide)
                BAR="["
                for ((j=0; j<40; j++)); do
                    if [ $j -lt $((STEP_PROGRESS * 40 / 100)) ]; then
                        BAR="${BAR}█"
                    else
                        BAR="${BAR}░"
                    fi
                done
                BAR="${BAR}]"
                
                # Get current statistics
                STATS=$(tc -s class show dev $INTERFACE 2>/dev/null | grep -A 2 "class htb 1:30" | tail -1 | sed 's/^[[:space:]]*//')
                
                echo -ne "\r${BAR} ${STEP_PROGRESS}% | Next: ${NEXT_MBPS} Mbps in ${i}s | Stats: ${STATS:0:40}..."
                sleep 1
            done
            echo ""  # New line
        fi
    done
    
    echo "========================================"
    echo "✓ Gradual increase complete! Final limit: 30 Mbps"
}

# Enhanced version with logging
enhanced_gradual_increase() {
    echo "Enhanced mode with logging"
    echo "Target: 30 Mbps, Zero period: ${ZERO_DURATION}s, Steps: $STEPS of ${STEP_DURATION}s"
    echo "========================================"
    
    # Create log file with timestamp
    LOG_FILE="/tmp/bandwidth_test_$(date +%Y%m%d_%H%M%S).log"
    echo "Logging to: $LOG_FILE"
    
    START_TIME=$(date +%s)
    
    # Log test parameters
    echo "BANDWIDTH TEST LOG" > $LOG_FILE
    echo "==================" >> $LOG_FILE
    echo "Start time: $(date)" >> $LOG_FILE
    echo "Interface: $INTERFACE" >> $LOG_FILE
    echo "Target: 30 Mbps" >> $LOG_FILE
    echo "Zero period: $ZERO_DURATION seconds" >> $LOG_FILE
    echo "Steps: $STEPS of $STEP_DURATION seconds each" >> $LOG_FILE
    echo "" >> $LOG_FILE
    
    # Phase 1: Zero period
    echo "[$(date '+%H:%M:%S')] PHASE 1: Zero bandwidth period started" | tee -a $LOG_FILE
    apply_limit "1kbit" 0 0 "zero"
    echo "[$(date '+%H:%M:%S')] Zero bandwidth applied" >> $LOG_FILE
    
    # Zero period countdown with logging
    for ((i=$ZERO_DURATION; i>0; i--)); do
        if [ $((i % 10)) -eq 0 ]; then  # Log every 10 seconds
            echo "[$(date '+%H:%M:%S')] Zero period: ${i}s remaining" >> $LOG_FILE
        fi
        PROGRESS=$(( (ZERO_DURATION - i) * 100 / ZERO_DURATION ))
        echo -ne "\rZero period: ${PROGRESS}% complete | ${i}s left..."
        sleep 1
    done
    echo ""
    echo "[$(date '+%H:%M:%S')] PHASE 1 complete" | tee -a $LOG_FILE
    echo "" >> $LOG_FILE
    
    # Phase 2: Gradual increase
    echo "[$(date '+%H:%M:%S')] PHASE 2: Gradual increase started" | tee -a $LOG_FILE
    
    for ((step=1; step<=STEPS; step++)); do
        CURRENT_MBPS=$((step * 5))
        CURRENT_LIMIT="${CURRENT_MBPS}mbit"
        STEP_START=$(date +%s)
        
        echo "[$(date '+%H:%M:%S')] Step $step/$STEPS: Setting to $CURRENT_MBPS Mbps" | tee -a $LOG_FILE
        apply_limit $CURRENT_LIMIT $step $CURRENT_MBPS "step"
        
        # Log the configuration
        echo "  Applied at: $(date '+%H:%M:%S')" >> $LOG_FILE
        tc -s class show dev $INTERFACE >> $LOG_FILE 2>&1
        echo "" >> $LOG_FILE
        
        if [ $step -lt $STEPS ]; then
            NEXT_MBPS=$(( (step + 1) * 5 ))
            
            # Step countdown with logging
            for ((i=$STEP_DURATION; i>0; i--)); do
                if [ $((i % 10)) -eq 0 ]; then  # Log every 10 seconds
                    STATS=$(tc -s class show dev $INTERFACE 2>/dev/null | grep -A 2 "class htb 1:30" | tail -1)
                    echo "[$(date '+%H:%M:%S')] Step $step: ${i}s to next increase | Stats: $STATS" >> $LOG_FILE
                fi
                STEP_PROGRESS=$(( (STEP_DURATION - i) * 100 / STEP_DURATION ))
                echo -ne "\rStep $step/$STEPS: ${STEP_PROGRESS}% | Next: ${NEXT_MBPS} Mbps in ${i}s"
                sleep 1
            done
            echo ""
        fi
    done
    
    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))
    
    echo "========================================"
    echo "✓ Test complete!" | tee -a $LOG_FILE
    echo "  Final limit: 30 Mbps" | tee -a $LOG_FILE
    echo "  Total duration: $DURATION seconds" | tee -a $LOG_FILE
    echo "  Log file: $LOG_FILE" | tee -a $LOG_FILE
    
    # Show summary
    echo "" | tee -a $LOG_FILE
    echo "Test Summary:" | tee -a $LOG_FILE
    echo "  Phase 1: Zero bandwidth for $ZERO_DURATION seconds" | tee -a $LOG_FILE
    echo "  Phase 2: $STEPS steps of $STEP_DURATION seconds each" | tee -a $LOG_FILE
    echo "  Bandwidth progression:" | tee -a $LOG_FILE
    for ((step=1; step<=STEPS; step++)); do
        echo "    Step $step: $((step * 5)) Mbps" | tee -a $LOG_FILE
    done
}

# Function to monitor and display current limit
show_status() {
    echo "=== Current TC Configuration for $INTERFACE ==="
    echo ""
    echo "=== QDISCs ==="
    sudo tc -s qdisc show dev $INTERFACE
    echo ""
    echo "=== CLASSES ==="
    sudo tc -s class show dev $INTERFACE 2>/dev/null
    echo ""
    echo "=== Current Limit ==="
    CURRENT_RATE=$(tc class show dev $INTERFACE 2>/dev/null | grep "class htb" | head -1 | grep -o "rate [^ ]*" | cut -d' ' -f2)
    if [ -n "$CURRENT_RATE" ]; then
        if [[ $CURRENT_RATE == *mbit ]]; then
            MBPS=${CURRENT_RATE%mbit}
            echo "Currently limited to: $MBPS Mbps"
        else
            echo "Currently limited to: $CURRENT_RATE"
        fi
    else
        echo "No limit currently active"
    fi
}

# Function to stop and reset
remove_bandwidth_limit() {
    echo "Removing bandwidth limits from interface $INTERFACE"
    remove_existing_config
    echo "✓ Bandwidth limits removed from $INTERFACE"
}

# Main script
case "$1" in
    start)
        # Immediate 30 Mbps
        ORIGINAL_LIMIT="30mbit"
        echo "Setting immediate 30 Mbps limit"
        remove_existing_config
        sleep 2
        apply_limit "30mbit" "final" 30 "step"
        ;;
    test)
        # Full test with zero period + gradual increase
        echo "Starting full bandwidth test:"
        echo "  Phase 1: Zero bandwidth for $ZERO_DURATION seconds"
        echo "  Phase 2: 0→30 Mbps in $STEPS steps of $STEP_DURATION seconds"
        echo "========================================"
        run_zero_period
        gradual_increase
        ;;
    enhanced)
        enhanced_gradual_increase
        ;;
    zero)
        # Just test zero period
        run_zero_period
        ;;
    stop)
        remove_bandwidth_limit
        ;;
    status)
        show_status
        ;;
    *)
        echo "Usage: $0 {test|enhanced|zero|start|stop|status}"
        echo "  test      - Full test: 40s zero + 6x40s steps to 30 Mbps"
        echo "  enhanced  - Same with detailed logging"
        echo "  zero      - Just test the zero bandwidth period"
        echo "  start     - Set 30 Mbps limit immediately"
        echo "  stop      - Remove bandwidth limits"
        echo "  status    - Show current configuration"
        exit 1
        ;;
esac