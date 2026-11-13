LOCAL_TS_IP='100.66.9.62'
PEER_TS_IP='100.74.118.59'

mkdir "$HOME/.config/ros"

cat > "$HOME/.config/ros/fastdds-tailscale.xml" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>tailscale_udp</transport_id>
      <type>UDPv4</type>
      <maxMessageSize>1200</maxMessageSize>
      <maxInitialPeersRange>64</maxInitialPeersRange>
      <interfaceWhiteList>
        <address>${LOCAL_TS_IP}</address>
        <address>127.0.0.1</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="tailscale_peer" is_default_profile="true">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator/>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>${PEER_TS_IP}</address>
            </udpv4>
          </locator>
          <locator>
            <udpv4>
              <address>127.0.0.1</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
      <userTransports>
        <transport_id>tailscale_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF

cat > "$HOME/.config/ros/tailscale-env.bash" <<'EOF'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/ros/fastdds-tailscale.xml"

unset ROS_DISCOVERY_SERVER
unset ROS_STATIC_PEERS
unset ROS_AUTOMATIC_DISCOVERY_RANGE
EOF

