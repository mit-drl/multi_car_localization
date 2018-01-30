# these are assigned with DHCP so you might have to change them occasionally
rc1ip=192.168.1.101
rc2ip=192.168.1.102
rc3ip=192.168.1.103

# mr -> “mount remote”
# to mount the home directory of racecar 77 on your local filesystem
# at location: ~/remote/racecar77
# mr77 ~/remote/racecar77
function mr1()  { sshfs racecar@$rc1ip:/home/racecar $1 -o ssh_command='sshpass -p racecar@mit ssh'; }
function mr2()  { sshfs racecar@$rc2ip:/home/racecar $1 -o ssh_command='sshpass -p racecar@mit ssh'; }
function mr3()  { sshfs racecar@$rc3ip:/home/racecar $1 -o ssh_command='sshpass -p racecar@mit ssh'; }

# rc -> “racecar …”
alias rc1="sshpass -p racecar@mit ssh racecar@$rc77ip"
alias rc2="sshpass -p racecar@mit ssh racecar@$rc66ip"
alias rc3="sshpass -p racecar@mit ssh racecar@$rc34ip"

# Make an alias for connecting ROS programs to the RACECAR cars.
runcar() {
    # Check that we have enough arguments
    if (( $# < 2 )) || ! [[ $1 =~ ^[0-9]+$ ]]; then
        echo "ERROR: Wrong arguments supplied. Please refer to the usage below."
        echo ""
        echo "USAGE:"
        echo "runcar <CAR_NUM> <COMMAND>"
        echo ""
        echo "EXAMPLE:"
        echo "$ runcar 4 rviz"
        echo "$ runcar 77 rostopic list"
        return -1
    fi
    
    # We have enough arguments. Proceed with executing command with remote ROS master.
    CAR_NUM="$1"
    CAR_IP="192.168.1.10$CAR_NUM"
    # Clean up input args
    shift

    # Allow for external ROS nodes to talk to us using our IP address.
    VM_IP=`hostname -I`
    
    # Check that our IP actually exists.
    if [[ -z "$VM_IP" ]]; then
        echo "ERROR: The VM does not seem to have an IP address or access to the internet. Are you connected via NAT and DHCP?"
        return -1
    fi

    # Construct URI
    CAR_ROS_MASTER_URI="http://$CAR_IP:11311"

    # Check that we can ping the car.
    ping -c1 -w1 "$CAR_IP" >> /dev/null
    if (( $? != 0 )); then
        echo "ERROR: could not ping the car at $CAR_IP. Is the car on and the VM is connected to the car's router?"
        return -1
    fi

    # Check that we can access the ros master.
    HTTP_RESPONSE=`curl --write-out "%{http_code}" --silent --output /dev/null "$CAR_ROS_MASTER_URI"`
        if (( HTTP_RESPONSE != 501 )); then
                echo "ERROR: could not access the car's roscore at $CAR_IP. Is roscore running on the car?"
                return -1
        fi
    
    # Execute command with ROS env args
    ROS_MASTER_URI="$CAR_ROS_MASTER_URI" ROS_IP="$VM_IP" "$@"

    if (( $? != 0)); then
        echo "ERROR: Check that you can ping 192.168.1.10$CARNUM and that roscore is running on the car."
    fi

    return "$?"
}
