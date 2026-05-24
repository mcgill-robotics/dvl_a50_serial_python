import crcmod

# Water Linked uses CRC-8 with polynomial 0x07, init 0x00
crc8_func = crcmod.predefined.mkPredefinedCrcFun("crc-8")

def verify_checksum(sentence: str) -> bool:
    """Verifies the *xx checksum at the end of a DVL message."""
    if "*" not in sentence:
        return False
    try:
        data_part, checksum_part = sentence.rsplit("*", 1)
        expected_crc = int(checksum_part.strip(), 16)
        calculated_crc = crc8_func(data_part.encode('utf-8'))
        return expected_crc == calculated_crc
    except ValueError:
        return False

def parse_wrz(sentence: str):
    """
    Parses a modern velocity report (wrz).
    Format: wrz,*[vx],[vy],[vz],[valid],[altitude],[fom],[covariance],[time_of_validity],[time_of_transmission],[time],[status]*[checksum]
    """
    # Remove the checksum part
    data_part = sentence.rsplit("*", 1)[0]
    parts = data_part.split(",")
    
    if len(parts) < 12:
        return None
        
    try:
        cov_list = []
        if len(parts) > 7 and parts[7]:
            cov_strs = parts[7].split(";")
            if len(cov_strs) == 9:
                cov_list = [float(x) for x in cov_strs]
                
        return {
            "vx": float(parts[1]),
            "vy": float(parts[2]),
            "vz": float(parts[3]),
            "valid": (parts[4] == 'y'),
            "altitude": float(parts[5]),
            "fom": float(parts[6]),
            "covariance": cov_list,
            "time_of_validity": int(parts[8]) if len(parts) > 8 else 0,
            "time_of_transmission": int(parts[9]) if len(parts) > 9 else 0,
            "time": float(parts[10]) if len(parts) > 10 else 0.0,
            "status": int(parts[11]) if len(parts) > 11 else 0
        }
    except ValueError:
        return None

def parse_wru(sentence: str):
    """
    Parses a transducer report (wru).
    Format: wru,[id],[velocity],[distance],[rssi],[nsd]*[checksum]
    """
    data_part = sentence.rsplit("*", 1)[0]
    parts = data_part.split(",")
    if len(parts) < 6:
        return None
    try:
        return {
            "id": int(parts[1]),
            "velocity": float(parts[2]),
            "distance": float(parts[3]),
            "rssi": float(parts[4]),
            "nsd": float(parts[5])
        }
    except ValueError:
        return None

def parse_wrp(sentence: str):
    """
    Parses a dead reckoning report (wrp).
    Format: wrp,[time_stamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],[status]*[checksum]
    """
    data_part = sentence.rsplit("*", 1)[0]
    parts = data_part.split(",")
    if len(parts) < 10:
        return None
    try:
        return {
            "time_stamp": float(parts[1]),
            "x": float(parts[2]),
            "y": float(parts[3]),
            "z": float(parts[4]),
            "pos_std": float(parts[5]),
            "roll": float(parts[6]),
            "pitch": float(parts[7]),
            "yaw": float(parts[8]),
            "status": int(parts[9])
        }
    except ValueError:
        return None

def parse_wrv(sentence: str):
    """
    Parses a version report (wrv).
    Format: wrv,[part_number],[version]*[checksum]
    """
    data_part = sentence.rsplit("*", 1)[0]
    parts = data_part.split(",")
    if len(parts) < 3:
        return None
    return {
        "part_number": parts[1],
        "version": parts[2]
    }

def parse_wrc(sentence: str):
    """
    Parses a config report (wrc).
    Format: wrc,[speed_of_sound],[mounting_rotation_offset],[acoustic_enabled],[dark_mode_enabled],[range_mode],[periodic_cycling_enabled]*[checksum]
    """
    data_part = sentence.rsplit("*", 1)[0]
    parts = data_part.split(",")
    if len(parts) < 5:
        return None
        
    config = {
        "speed_of_sound": float(parts[1]),
        "mounting_rotation_offset": float(parts[2]),
        "acoustic_enabled": (parts[3] == 'y'),
        "dark_mode_enabled": (parts[4] == 'y'),
        "range_mode": parts[5] if len(parts) > 5 else "auto",
        "periodic_cycling_enabled": (parts[6] == 'y') if len(parts) > 6 else False
    }
    return config
