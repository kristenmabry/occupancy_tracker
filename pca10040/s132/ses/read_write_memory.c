#include "sdk_common.h"
#include "read_write_memory.h"
#include <string.h>
#include "nrf_log.h"




void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != NRF_SUCCESS)
            {
                // Initialization failed.
            }
            break;
				case FDS_EVT_WRITE:
						if (p_fds_evt->result == NRF_SUCCESS)
						{
							write_flag_fds_test=1;
						}
						break;
        default:
            break;
    }
}


ret_code_t fds_test_write(void)
{
		
		//static uint32_t const m_deadbeef[2] = {0xDEADBEEF,0xBAADF00D};
		static uint8_t const m_deadbeef[4] = {0x1,0x2,0x3,0x4};
		fds_record_t        record;
		fds_record_desc_t   record_desc;

		// Set up data.
		
		// Set up record.
		record.file_id              = FILE_ID_FDS_TEST;
		record.key              		= REC_KEY_FDS_TEST;
		record.data.p_data       = &m_deadbeef;
		//record.data.length_words   = sizeof(m_deadbeef)/sizeof(uint32_t);
		record.data.length_words   = sizeof(m_deadbeef)/sizeof(uint8_t);
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != NRF_SUCCESS)
		{
				return ret;
		}
		 NRF_LOG_INFO("Writing Record ID = %d \r\n",record_desc.record_id);
		return NRF_SUCCESS;
}


ret_code_t fds_test_read(void)
{

		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		//uint32_t *data;
		uint8_t *data;
		uint32_t err_code;
		
		NRF_LOG_INFO("Start searching... \r\n");
		// Loop until all records with the given key and file ID have been found.
		while (fds_record_find(FILE_ID_FDS_TEST, REC_KEY_FDS_TEST, &record_desc, &ftok) == NRF_SUCCESS)
		{
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != NRF_SUCCESS)
				{
					return err_code;		
				}
				
				NRF_LOG_INFO("Found Record ID = %d\r\n",record_desc.record_id);
				NRF_LOG_INFO("Data = ");
				//data = (uint32_t *) flash_record.p_data;
				data = (uint8_t *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->length_words;i++)
				{
					NRF_LOG_INFO("0x%8x ",data[i]);
				}
				NRF_LOG_INFO("\r\n");
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);
				if (err_code != NRF_SUCCESS)
				{
					return err_code;	
				}
		}
		return NRF_SUCCESS;
		
}


ret_code_t kls_fds_write(uint32_t write_file_id, uint32_t write_record_key , uint8_t write_data[])
{		
		static uint8_t m_deadbeef[10] = {0};
		
		memcpy(m_deadbeef, write_data, sizeof(m_deadbeef));
		
		fds_record_t        record;
		fds_record_desc_t   record_desc;
	
		// Set up record.
		record.file_id              = write_file_id;
		record.key              		= write_record_key;
		record.data.p_data       		= &m_deadbeef;
		record.data.length_words   	= sizeof(m_deadbeef)/sizeof(uint8_t);
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != NRF_SUCCESS)
		{
				return ret;
		}
		 NRF_LOG_INFO("Writing Record ID = %d \r\n",record_desc.record_id);
		return NRF_SUCCESS;
	
}
	

ret_code_t kls_fds_read(uint32_t read_file_id, uint32_t relevant_record_key , uint8_t read_data[])
{	
		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint8_t *data;
		uint32_t err_code;
		
		NRF_LOG_INFO("Start searching... \r\n");
		// Loop until all records with the given key and file ID have been found.
		while (fds_record_find(read_file_id, relevant_record_key, &record_desc, &ftok) == NRF_SUCCESS)
		{
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != NRF_SUCCESS)
				{
					return err_code;		
				}
				
				NRF_LOG_INFO("Found Record ID = %d\r\n",record_desc.record_id);

				data = (uint8_t *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->length_words;i++)
				{
					read_data[i] = data[i];
				}
			
		
				NRF_LOG_HEXDUMP_INFO(read_data, sizeof(read_data));
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);
				if (err_code != NRF_SUCCESS)
				{
					return err_code;	
				}
		}
		return NRF_SUCCESS;	
}


ret_code_t fds_test_find_and_delete (void)
{

		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	
		ftok.page=0;
		ftok.p_addr=NULL;
		// Loop and find records with same ID and rec key and mark them as deleted. 
		while (fds_record_find(FILE_ID_FDS_TEST, REC_KEY_FDS_TEST, &record_desc, &ftok) == NRF_SUCCESS)
		{
			fds_record_delete(&record_desc);
			NRF_LOG_INFO("Deleted record ID: %d \r\n",record_desc.record_id);
		}
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret_code_t ret = fds_gc();
		if (ret != NRF_SUCCESS)
		{
				return ret;
		}
		return NRF_SUCCESS;
}


ret_code_t kls_fds_find_and_delete (uint32_t read_file_id, uint32_t relevant_record_key)
{

		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	
		ftok.page=0;
		ftok.p_addr=NULL;
		// Loop and find records with same ID and rec key and mark them as deleted. 
		while (fds_record_find(read_file_id, relevant_record_key, &record_desc, &ftok) == NRF_SUCCESS)
		{
			fds_record_delete(&record_desc);
			NRF_LOG_INFO("Deleted record ID: %d \r\n",record_desc.record_id);
		}
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret_code_t ret = fds_gc();
		if (ret != NRF_SUCCESS)
		{
				return ret;
		}
		return NRF_SUCCESS;
}


ret_code_t fds_test_init (void)
{
	
		ret_code_t ret = fds_register(my_fds_evt_handler);
		if (ret != NRF_SUCCESS)
		{
					return ret;
				
		}
		ret = fds_init();
		if (ret != NRF_SUCCESS)
		{
				return ret;
		}
		
		return NRF_SUCCESS;
		
}