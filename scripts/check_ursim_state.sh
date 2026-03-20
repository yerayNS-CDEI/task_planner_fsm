#!/bin/bash
# URSim State Check Script
# Run this before starting the FSM to ensure clean URSim state

echo "=== URSim State Check ==="
echo ""

# Check if URSim is running
echo "1. Checking if URSim is running..."
URSIM_PID=$(pgrep -f "URControl.*ur10e" | head -1)
if [ -z "$URSIM_PID" ]; then
    echo "   ✗ URSim NOT running"
    echo "   → Start it with: ros2 run ur_client_library start_ursim.sh -m ur10e -v 5.17.3"
else
    echo "   ✓ URSim running (PID: $URSIM_PID)"
fi
echo ""

# Check URSim port connectivity
echo "2. Checking URSim ports..."
for port in 29999 30001 30002 30003 30004; do
    if timeout 1 bash -c "echo > /dev/tcp/192.168.56.101/$port" 2>/dev/null; then
        echo "   ✓ Port $port: Open"
    else
        echo "   ✗ Port $port: Closed/Unreachable"
    fi
done
echo ""

# Check for existing arm driver connections
echo "3. Checking for existing arm driver nodes..."
ARM_NODES=$(ros2 node list 2>/dev/null | grep -E "ur_ros2_control|dashboard_client|controller_manager" || true)
if [ -z "$ARM_NODES" ]; then
    echo "   ✓ No existing arm driver nodes"
else
    echo "   ⚠ Found existing arm driver nodes:"
    echo "$ARM_NODES" | sed 's/^/     /'
    echo "   → May cause conflicts. Consider killing: pkill -f ur_ros2_control"
fi
echo ""

# Check TF static publishers
echo "4. Checking static TF publishers..."
STATIC_TF=$(ros2 node list 2>/dev/null | grep static_transform || true)
if [ -z "$STATIC_TF" ]; then
    echo "   ✓ No static TF publishers (expected before launching)"
else
    echo "   Found static TF publishers:"
    echo "$STATIC_TF" | sed 's/^/     /'
fi
echo ""

# Check /clock topic
echo "5. Checking /clock topic..."
if ros2 topic info /clock &>/dev/null; then
    CLOCK_HZ=$(ros2 topic hz /clock --window 10 2>&1 | grep "average rate" | awk '{print $3}' || echo "N/A")
    echo "   ✓ /clock topic active (rate: $CLOCK_HZ Hz)"
else
    echo "   ✗ /clock topic not available (will be created when simulation starts)"
fi
echo ""

# Recommendations
echo "=== Recommendations ==="
if [ -z "$URSIM_PID" ]; then
    echo "❌ URSim must be running before starting FSM"
    echo "   Start: ros2 run ur_client_library start_ursim.sh -m ur10e -v 5.17.3"
    echo "   Wait: 10-15 seconds for URSim GUI to fully initialize"
else
    echo "✓ URSim is running"
    if [ -n "$ARM_NODES" ]; then
        echo "⚠ Consider cleaning up old arm nodes:"
        echo "   pkill -f ur_ros2_control"
        echo "   pkill -f dashboard_client"
    fi
    echo ""
    echo "Ready to start FSM!"
fi
echo ""
