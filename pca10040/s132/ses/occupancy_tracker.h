

struct 
{
  uint8_t current_occupancy;
  uint16_t ceiling_heigh;
  uint8_t isReady;
} occupancy_data;

struct
{
uint8_t person_entry, 
        check , 
        person_exit , 
        person_entry_2, 
        person_exit_2;
} detection_data;