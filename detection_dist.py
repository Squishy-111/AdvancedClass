"""
I made this code so we can find the detection distance for an april tag and test out different focal lengths and fov

"""



def calculate_max_range(tag_size_meters, video_width_pixels, min_tag_width_px=25):
    
    # Constants for RPi Camera Module 3
    FOCAL_LENGTH_MM = 4.74
    SENSOR_WIDTH_MM = 6.45 
    
    # Calculate Focal Length in Pixels for this specific video resolution
    focal_length_px = (FOCAL_LENGTH_MM * video_width_pixels) / SENSOR_WIDTH_MM
    
    # Calculate Max Distance(min_tag_width_px is how much pixels needed to detect tag and changes based on blur and software)
    max_distance = (focal_length_px * tag_size_meters) / min_tag_width_px
    
    return max_distance

# Example: 640x480 video, 15cm tag width measured from black square edge
range_limit = calculate_max_range(0.15, 640)
print(f"Max Detection Range: {range_limit:.2f} meters")