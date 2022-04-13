/*
*
*/
#ifndef RWM_H__
#define RWM_H__


#include <stdint.h>
#include <stdbool.h>
#include "fds.h"

/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
//#define RWM_DEF(_name)                                                                          \
//static rwm_t _name;                                                                             \

// Forward declaration of the ble_cus_t type.
//typedef struct rwm_s rwm_t;



static volatile uint8_t write_flag_fds_test = 0;
#define FILE_ID_FDS_TEST     0x1111
#define REC_KEY_FDS_TEST     0x2222

#define OCCU_FILE_ID_FDS      0x1234
#define OCCU_REC_KEY_FDS      0x4321     

#define CEIL_FILE_ID_FDS      0x1000
#define CEIL_REC_KEY_FDS      0x1616

void my_fds_evt_handler(fds_evt_t const * const p_fds_evt);


ret_code_t fds_test_write(void);


ret_code_t fds_test_read(void);


ret_code_t kls_fds_write(uint32_t write_file_id, uint32_t write_record_key , uint8_t write_data[]);


ret_code_t kls_fds_read(uint32_t read_file_id, uint32_t relevant_record_key , uint8_t read_data[]);


ret_code_t fds_test_find_and_delete (void);


ret_code_t kls_fds_find_and_delete (uint32_t read_file_id, uint32_t relevant_record_key);



ret_code_t fds_test_init (void);
#endif // RWM_H__