#!/bin/bash

# Bandwidth Limiter Script with Initial Zero Period
INTERFACE="wlan0"
TARGET_THROUGHPUT="30"  # Target application throughput in Mbps
BURST="32k"
STEPS=6  # Number of steps after zero period
STEP_DURATION=40  # Seconds between steps
ZERO_DURATION=40  # Initial zero bandwidth period

# Overhead compensation (typical network overhead ~13-15%)
# Adjust this based on your network conditions
OVERHEAD_PERCENT=15  # 15% overhead compensation

# Calculate adjusted limits to achieve desired throughput
calculate_adjusted_limit() {
    local target_mbps=$1
    if [ "$target_mbps" -eq 0 ]; then
        echo "1kbit"  # Near-zero for zero period
    else
        # Add overhead compensation
        local adjusted=$((target_mbps * (100 + OVERHEAD_PERCENT) / 100))
        echo "${adjusted}mbit"
    fi
}

echo "=========================================="
echo "Bandwidth Limiter Script"
echo "=========================================="
echo "Working with interface: $INTERFACE"
echo "Target application throughput: $TARGET_THROUGHPUT Mbps"
echo "Overhead compensation: $OVERHEAD_PERCENT%"
echo "Initial zero period: $ZERO_DURATION seconds"
echo "Steps after zero: $STEPS steps of $STEP_DURATION seconds each"
echo "Bandwidth progression: 0 → 5 → 10 → 15 → 20 → 25 → 30 Mbps (throughput)"
echo "=========================================="

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

# Enhanced function to apply bandwidth limit with better HTB configuration
apply_limit() {
    local target_throughput=$1
    local step_num=$2
    local phase=$3  # "zero" or "step"
    
    # Calculate the actual tc limit with overhead compensation
    local tc_limit=$(calculate_adjusted_limit $target_throughput)
    
    echo ""
    if [ "$phase" = "zero" ]; then
        echo "=== ZERO BANDWIDTH PHASE ==="
        echo "  Target: 0 Mbps throughput"
        echo "  TC limit: $tc_limit (near-zero)"
    else
        echo "=== Step $step_num/$STEPS: $target_throughput Mbps throughput ==="
        echo "  TC limit: $tc_limit (compensated for $OVERHEAD_PERCENT% overhead)"
    fi
    
    # Remove existing config for this step
    sudo tc qdisc del dev $INTERFACE root 2>/dev/null
    
    # Apply new limit with better HTB configuration
    # Using htb with appropriate burst and cburst for smoother traffic
    sudo tc qdisc add dev $INTERFACE root handle 1: htb default 30 r2q 10
    
    # Calculate appropriate burst sizes based on rate
    local burst_size="${tc_limit%mbit}kb"
    if [ "${tc_limit%mbit}" -lt 10 ]; then
        burst_size="16k"  # Smaller burst for low bandwidth
    elif [ "${tc_limit%mbit}" -lt 20 ]; then
        burst_size="32k"  # Medium burst
    else
        burst_size="64k"  # Larger burst for high bandwidth
    fi
    
    # Main class with rate limit
    sudo tc class add dev $INTERFACE parent 1: classid 1:1 htb rate $tc_limit burst $burst_size cburst $burst_size
    
    # Default class for all traffic
    sudo tc class add dev $INTERFACE parent 1:1 classid 1:30 htb rate $tc_limit burst $burst_size cburst $burst_size ceil $tc_limit
    
    # Add filter to match all IP traffic
    sudo tc filter add dev $INTERFACE protocol ip parent 1: prio 1 u32 match ip protocol 0 0xff flowid 1:30
    
    # Add fq_codel qdisc for better fairness and lower latency
    sudo tc qdisc add dev $INTERFACE parent 1:30 fq_codel
    
    if [ "$phase" = "zero" ]; then
        echo "✓ Applied ZERO bandwidth limit (effective: ~0 Mbps)"
    else
        echo "✓ Applied limit: $target_throughput Mbps throughput target"
        echo "  TC rate: $tc_limit (includes overhead compensation)"
    fi
    
    # Show current configuration
    echo "  Active configuration:"
    tc class show dev $INTERFACE 2>/dev/null | grep -o "rate [^ ]*" | head -1
}

# Function to measure actual throughput
measure_throughput() {
    local duration=$1
    local interface=$2
    
    # Get initial byte count
    local rx_bytes_start=$(cat /sys/class/net/$interface/statistics/rx_bytes 2>/dev/null || echo 0)
    local tx_bytes_start=$(cat /sys/class/net/$interface/statistics/tx_bytes 2>/dev/null || echo 0)
    
    sleep $duration
    
    # Get final byte count
    local rx_bytes_end=$(cat /sys/class/net/$interface/statistics/rx_bytes 2>/dev/null || echo 0)
    local tx_bytes_end=$(cat /sys/class/net/$interface/statistics/tx_bytes 2>/dev/null || echo 0)
    
    # Calculate throughput in Mbps
    local rx_bytes=$((rx_bytes_end - rx_bytes_start))
    local tx_bytes=$((tx_bytes_end - tx_bytes_start))
    local total_bytes=$((rx_bytes + tx_bytes))
    local total_bits=$((total_bytes * 8))
    local throughput_mbps=$(echo "scale=2; $total_bits / $duration / 1000000" | bc 2>/dev/null || echo "0")
    
    echo $throughput_mbps
}

# Function to run zero bandwidth period
run_zero_period() {
    echo ""
    echo "========================================"
    echo "PHASE 1: INITIAL ZERO BANDWIDTH PERIOD"
    echo "========================================"
    echo "Setting bandwidth to 0 for $ZERO_DURATION seconds"
    
    # Apply zero limit
    apply_limit 0 0 "zero"
    
    echo "----------------------------------------"
    echo "Zero bandwidth period started. Countdown:"
    
    # Countdown for zero period with throughput monitoring
    for ((i=$ZERO_DURATION; i>0; i--)); do
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
        
        echo -ne "\r${BAR} ${PROGRESS}% | Time left: ${i}s"
        sleep 1
    done
    echo ""  # New line
    echo "✓ Zero bandwidth period complete!"
}

# Function to gradually increase bandwidth
gradual_increase() {
    echo ""
    echo "========================================"
    echo "PHASE 2: GRADUAL BANDWIDTH INCREASE"
    echo "========================================"
    echo "Increasing throughput from 0 to $TARGET_THROUGHPUT Mbps in $STEPS steps"
    echo "Each step: +5 Mbps throughput every $STEP_DURATION seconds"
    echo "Note: TC limits are adjusted +$OVERHEAD_PERCENT% to compensate for overhead"
    
    # Step progression for throughput (5 Mbps increments)
    for ((step=1; step<=STEPS; step++)); do
        # Calculate target throughput for this step (5, 10, 15, 20, 25, 30)
        TARGET_THROUGHPUT_STEP=$((step * 5))
        
        apply_limit $TARGET_THROUGHPUT_STEP $step "step"
        
        if [ $step -lt $STEPS ]; then
            NEXT_THROUGHPUT=$(( (step + 1) * 5 ))
            NEXT_TC_LIMIT=$(calculate_adjusted_limit $NEXT_THROUGHPUT)
            
            echo "----------------------------------------"
            echo "Current step: $TARGET_THROUGHPUT_STEP Mbps throughput"
            echo "Next step: $NEXT_THROUGHPUT Mbps throughput (TC limit: $NEXT_TC_LIMIT)"
            echo "Time until next increase: $STEP_DURATION seconds"
            
            # Countdown timer with progress bar and throughput monitoring
            for ((i=$STEP_DURATION; i>0; i--)); do
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
                
                # Measure actual throughput every 10 seconds
                if [ $((i % 10)) -eq 0 ] || [ $i -eq 1 ]; then
                    ACTUAL_THROUGHPUT=$(measure_throughput 1 $INTERFACE)
                    echo -ne "\r${BAR} ${STEP_PROGRESS}% | Time: ${i}s | Target: ${TARGET_THROUGHPUT_STEP} Mbps | Actual: ${ACTUAL_THROUGHPUT} Mbps"
                else
                    echo -ne "\r${BAR} ${STEP_PROGRESS}% | Time: ${i}s | Target: ${TARGET_THROUGHPUT_STEP} Mbps"
                fi
                sleep 1
            done
            echo ""  # New line
        fi
    done
    
    echo "========================================"
    echo "✓ Gradual increase complete! Final throughput target: $TARGET_THROUGHPUT Mbps"
}

# Enhanced version with comprehensive logging
enhanced_gradual_increase() {
    echo "Enhanced mode with detailed logging"
    echo "Target throughput: $TARGET_THROUGHPUT Mbps, Zero period: ${ZERO_DURATION}s"
    echo "Overhead compensation: $OVERHEAD_PERCENT%"
    echo "Steps: $STEPS of ${STEP_DURATION}s each"
    echo "========================================"
    
    # Create log file with timestamp
    LOG_FILE="/tmp/bandwidth_test_$(date +%Y%m%d_%H%M%S).log"
    echo "Logging to: $LOG_FILE"
    
    START_TIME=$(date +%s)
    
    # Log test parameters
    {
        echo "BANDWIDTH TEST LOG"
        echo "=================="
        echo "Start time: $(date)"
        echo "Interface: $INTERFACE"
        echo "Target throughput: $TARGET_THROUGHPUT Mbps"
        echo "Overhead compensation: $OVERHEAD_PERCENT%"
        echo "Zero period: $ZERO_DURATION seconds"
        echo "Steps: $STEPS of $STEP_DURATION seconds each"
        echo ""
        echo "Throughput Progression (Target vs Actual):"
        echo "----------------------------------------"
    } > $LOG_FILE
    
    # Phase 1: Zero period
    echo "[$(date '+%H:%M:%S')] PHASE 1: Zero bandwidth period started" | tee -a $LOG_FILE
    apply_limit 0 0 "zero"
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
        TARGET_THROUGHPUT_STEP=$((step * 5))
        TC_LIMIT=$(calculate_adjusted_limit $TARGET_THROUGHPUT_STEP)
        STEP_START=$(date +%s)
        
        echo "[$(date '+%H:%M:%S')] Step $step/$STEPS: Target throughput: $TARGET_THROUGHPUT_STEP Mbps (TC limit: $TC_LIMIT)" | tee -a $LOG_FILE
        apply_limit $TARGET_THROUGHPUT_STEP $step "step"
        
        # Log the configuration
        echo "  Applied at: $(date '+%H:%M:%S')" >> $LOG_FILE
        tc -s class show dev $INTERFACE >> $LOG_FILE 2>&1
        echo "" >> $LOG_FILE
        
        if [ $step -lt $STEPS ]; then
            NEXT_THROUGHPUT=$(( (step + 1) * 5 ))
            NEXT_TC_LIMIT=$(calculate_adjusted_limit $NEXT_THROUGHPUT)
            
            # Step countdown with logging and throughput measurement
            for ((i=$STEP_DURATION; i>0; i--)); do
                if [ $((i % 10)) -eq 0 ] || [ $i -eq 1 ]; then  # Log every 10 seconds and at the end
                    ACTUAL_THROUGHPUT=$(measure_throughput 1 $INTERFACE)
                    echo "[$(date '+%H:%M:%S')] Step $step: ${i}s to next increase | Target: ${TARGET_THROUGHPUT_STEP} Mbps | Actual: ${ACTUAL_THROUGHPUT} Mbps" >> $LOG_FILE
                    
                    # Log to summary as well
                    echo "  $TARGET_THROUGHPUT_STEP,$ACTUAL_THROUGHPUT,$(date '+%H:%M:%S')" >> ${LOG_FILE%.log}_data.csv
                fi
                
                STEP_PROGRESS=$(( (STEP_DURATION - i) * 100 / STEP_DURATION ))
                BAR="["
                for ((j=0; j<40; j++)); do
                    if [ $j -lt $((STEP_PROGRESS * 40 / 100)) ]; then
                        BAR="${BAR}█"
                    else
                        BAR="${BAR}░"
                    fi
                done
                BAR="${BAR}]"
                
                if [ $((i % 10)) -eq 0 ] || [ $i -eq 1 ]; then
                    echo -ne "\r${BAR} ${STEP_PROGRESS}% | Step $step: ${TARGET_THROUGHPUT_STEP} Mbps | Actual: ${ACTUAL_THROUGHPUT} Mbps | Next: ${NEXT_THROUGHPUT} Mbps in ${i}s"
                else
                    echo -ne "\r${BAR} ${STEP_PROGRESS}% | Step $step: ${TARGET_THROUGHPUT_STEP} Mbps | Next: ${NEXT_THROUGHPUT} Mbps in ${i}s"
                fi
                sleep 1
            done
            echo ""
        fi
    done
    
    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))
    
    # Create summary
    {
        echo ""
        echo "========================================"
        echo "TEST COMPLETE"
        echo "========================================"
        echo "Final target throughput: $TARGET_THROUGHPUT Mbps"
        echo "Overhead compensation used: $OVERHEAD_PERCENT%"
        echo "Total duration: $DURATION seconds"
        echo ""
        echo "Step-by-Step Results:"
        echo "Step | Target (Mbps) | Actual (Mbps) | Difference | Efficiency"
        echo "-----|----------------|---------------|------------|-----------"
        
        # Parse the CSV data for summary
        if [ -f "${LOG_FILE%.log}_data.csv" ]; then
            while IFS=',' read -r target actual time; do
                diff=$(echo "$actual - $target" | bc 2>/dev/null || echo "0")
                efficiency=$(echo "scale=1; $actual * 100 / $target" | bc 2>/dev/null || echo "0")
                printf "%4s | %14s | %13s | %10.1f | %5.1f%%\n" "" "$target" "$actual" "$diff" "$efficiency"
            done < "${LOG_FILE%.log}_data.csv"
        fi
        
        echo ""
        echo "Recommendation: Based on the results, you may want to adjust"
        echo "the OVERHEAD_PERCENT variable (currently $OVERHEAD_PERCENT%)"
        echo "to achieve more accurate throughput targets."
        
    } | tee -a $LOG_FILE
    
    echo ""
    echo "✓ Test complete! Log file: $LOG_FILE"
    echo "  Data file: ${LOG_FILE%.log}_data.csv"
}

# Function to monitor and display current limit with throughput
show_status() {
    echo "=== Current TC Configuration for $INTERFACE ==="
    echo ""
    echo "=== QDISCs ==="
    sudo tc -s qdisc show dev $INTERFACE
    echo ""
    echo "=== CLASSES ==="
    sudo tc -s class show dev $INTERFACE 2>/dev/null
    echo ""
    echo "=== Current Limits ==="
    CURRENT_RATE=$(tc class show dev $INTERFACE 2>/dev/null | grep "class htb" | head -1 | grep -o "rate [^ ]*" | cut -d' ' -f2)
    if [ -n "$CURRENT_RATE" ]; then
        if [[ $CURRENT_RATE == *mbit ]]; then
            MBPS=${CURRENT_RATE%mbit}
            ESTIMATED_THROUGHPUT=$(echo "scale=1; $MBPS * 100 / (100 + $OVERHEAD_PERCENT)" | bc)
            echo "TC rate limit: $MBPS Mbps"
            echo "Estimated throughput: ~${ESTIMATED_THROUGHPUT} Mbps (with $OVERHEAD_PERCENT% overhead)"
            
            # Measure actual throughput over 2 seconds
            echo "Measuring actual throughput..."
            ACTUAL=$(measure_throughput 2 $INTERFACE)
            echo "Current actual throughput: ${ACTUAL} Mbps"
        else
            echo "Currently limited to: $CURRENT_RATE"
        fi
    else
        echo "No limit currently active"
    fi
}

# Function to calibrate overhead
calibrate_overhead() {
    echo "=== Calibrating Network Overhead ==="
    echo "This will help determine the optimal OVERHEAD_PERCENT value"
    echo "Please ensure there is active network traffic during calibration"
    echo ""
    
    # Set a known limit
    local test_limit="20mbit"
    echo "Setting test limit to $test_limit..."
    remove_existing_config
    apply_limit 20 1 "calibration"
    
    echo "Measuring actual throughput over 10 seconds..."
    sleep 2  # Allow stabilization
    
    local measured=$(measure_throughput 10 $INTERFACE)
    local limit_num=20  # 20 Mbps target
    
    # Calculate actual overhead
    local actual_overhead=$(echo "scale=1; (20 - $measured) * 100 / $measured" | bc 2>/dev/null || echo "15")
    
    echo ""
    echo "=== Calibration Results ==="
    echo "TC limit set: 20 Mbps"
    echo "Measured throughput: $measured Mbps"
    echo "Calculated overhead: $actual_overhead%"
    echo ""
    echo "Recommendation: Set OVERHEAD_PERCENT=$actual_overhead in the script"
    
    # Offer to update
    read -p "Update OVERHEAD_PERCENT to $actual_overhead%? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sed -i "s/OVERHEAD_PERCENT=.*/OVERHEAD_PERCENT=$actual_overhead/" "$0"
        echo "✓ OVERHEAD_PERCENT updated to $actual_overhead%"
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
        # Immediate target throughput
        echo "Setting immediate $TARGET_THROUGHPUT Mbps throughput limit"
        TC_LIMIT=$(calculate_adjusted_limit $TARGET_THROUGHPUT)
        echo "TC limit: $TC_LIMIT (compensated for $OVERHEAD_PERCENT% overhead)"
        remove_existing_config
        sleep 2
        apply_limit $TARGET_THROUGHPUT 6 "final"
        ;;
    test)
        # Full test with zero period + gradual increase
        echo "Starting full bandwidth test:"
        echo "  Phase 1: Zero bandwidth for $ZERO_DURATION seconds"
        echo "  Phase 2: 0→$TARGET_THROUGHPUT Mbps throughput in $STEPS steps"
        echo "  Each step: +5 Mbps throughput every $STEP_DURATION seconds"
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
    calibrate)
        calibrate_overhead
        ;;
    stop)
        remove_bandwidth_limit
        ;;
    status)
        show_status
        ;;
    *)
        echo "Usage: $0 {test|enhanced|zero|calibrate|start|stop|status}"
        echo "  test      - Full test: zero period + gradual increase to target"
        echo "  enhanced  - Full test with detailed logging"
        echo "  zero      - Just test the zero bandwidth period"
        echo "  calibrate - Calibrate overhead for your network"
        echo "  start     - Set target throughput immediately"
        echo "  stop      - Remove bandwidth limits"
        echo "  status    - Show current configuration"
        exit 1
        ;;
esac