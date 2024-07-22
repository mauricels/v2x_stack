# BTP Type
uint8 btp_type

# GN Packet Transport type
uint8 pkt_transport

# GN Traffic class
uint8 traffic_class

# GN Maximum Packet Lifetime (seconds)
uint8 max_pkt_life_time

# BTP Destination port (BTP-A & BTP-B BTPTypes)
uint16 dest_port

# BTP Additional Port Information
## Source Port (BTP-A)
uint16 src_port
## Destination Port Info (BTP-B)
uint16 dest_info

# Location for GeoUnicast
Location location

# Area for GBC or GAC
Area area


# GN Security Type
uint8 sec_profile

# For Secure Packets (SecProfile != UDPRX_SEC_NONE):
# Result of Security Parsing @sa eC2XSEC_ParserResult
uint8 parser_result
uint8 verification_result
uint8 ssp_len
uint32 aid

# SSP Bits/Data
uint8[32] ssp_bits

# CertID8
uint8[32] cert_id

# The length in octets of the 'on-the-air' frame data
uint16 length
