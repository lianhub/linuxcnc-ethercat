//
//    Copyright (C) 2014 Sascha Ittner <sascha.ittner@modusoft.de>
//

#include "lcec.h"
#include "lcec_ax5000.h"

typedef struct {
  int do_init;
  //hal_float_t *vel_fb;
 
  unsigned int control_pdo_0;
  unsigned int cmdvel_pdo_0;
  unsigned int status_pdo_0;
  unsigned int currpos_pdo_0;
  unsigned int control_pdo_1;
  unsigned int cmdvel_pdo_1;
  unsigned int status_pdo_1;
  unsigned int currpos_pdo_1;
  
} lcec_ax5000_data_t;

static ec_pdo_entry_info_t ax5000_pdo_entries[] = {
    {134, 0,  16}, // master control word          (MDT1)
    {36,  0,  32}, // velocity command value
    {134, 0,  16}, // master control word          (MDT2)
    {36,  0,  32}, // velocity command value
    {135, 0,  16}, // drive status word            (AT1)
    {51,  0,  32}, // position feedback value
    {135, 0,  16}, // drive status word            (AT2)
    {51,  0,  32}, // position feedback value
};

static ec_pdo_info_t ax5000_pdos[] = {
    {0x0018, 2, ax5000_pdo_entries},
    {0x1018, 2, ax5000_pdo_entries + 2},
    {0x0010, 2, ax5000_pdo_entries + 4},
    {0x1010, 2, ax5000_pdo_entries + 6}    
};

static ec_sync_info_t lcec_ax5000_syncs[] = {
    {2, EC_DIR_OUTPUT,2, ax5000_pdos},
    {3, EC_DIR_INPUT, 2, ax5000_pdos +2},
    {0xff}
};

//void lcec_ax5000_read(struct lcec_slave *slave, long period);
void lcec_ax5000_write(struct lcec_slave *slave, long period);

int lcec_ax5000_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_ax5000_data_t *hal_data;
  int err;
  uint32_t tu;
  int8_t ti;

  // initialize callbacks
  //slave->proc_read = lcec_ax5000_read;
  slave->proc_write = lcec_ax5000_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_ax5000_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_ax5000_data_t));
  slave->hal_data = hal_data;

  // initialize sync info
  slave->sync_info = lcec_ax5000_syncs;

  // initialize POD entries
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 134, 0x00, &hal_data->control_pdo_0, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid,  36, 0x00, &hal_data->cmdvel_pdo_0,  NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 135, 0x00, &hal_data->status_pdo_0,  NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid,  51, 0x00, &hal_data->currpos_pdo_0, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 134, 0x00, &hal_data->control_pdo_1, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid,  36, 0x00, &hal_data->cmdvel_pdo_1,  NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 135, 0x00, &hal_data->status_pdo_1,  NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid,  51, 0x00, &hal_data->currpos_pdo_1, NULL);

  //config_idns(sc);
  //ecrt_slave_config_dc(sc, 0x0730, 0x0003d090, 0, 0x001ab3f0, 0); 
	/*
  // export pins
  if ((err = hal_pin_float_newf(HAL_OUT, &(hal_data->vel_fb), comp_id, "%s.%s.%s.srv-vel-fb", LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exporting pin %s.%s.%s.srv-vel-fb failed\n", LCEC_MODULE_NAME, master->name, slave->name);
    return err;
  }
  // set default pin values
  *(hal_data->vel_fb) = 0.0;
 */
  // initialize variables
  hal_data->do_init = 0;
  
  return 0;
}
/*
void lcec_ax5000_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ax5000_data_t *hal_data = (lcec_ax5000_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  int32_t speed_raw;  
  int32_t pos_cnt, pos_cnt_diff;

  // read status word
  status =    EC_READ_U16(&pd[hal_data->status_pdo_os]);
  speed_raw = EC_READ_S32(&pd[hal_data->currvel_pdo_os]);
  pos_cnt =   EC_READ_S32(&pd[hal_data->currpos_pdo_os]);
  pos_cnt =   EC_READ_S32(&pd[hal_data->extenc_pdo_os]);
}
*/
void lcec_ax5000_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ax5000_data_t *hal_data = (lcec_ax5000_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  int toggle;
  //uint16_t control;  double speed_raw;

  //EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);
  toggle = hal_data->do_init;
  if(toggle==0) EC_WRITE_U16(&pd[hal_data->control_pdo_0+0], 0xe000);
  else          EC_WRITE_U16(&pd[hal_data->control_pdo_0+0], 0xe400);
  if(toggle==0) EC_WRITE_U16(&pd[hal_data->control_pdo_1+6], 0xe000);
  else          EC_WRITE_U16(&pd[hal_data->control_pdo_1+6], 0xe400);
  toggle = !toggle; 
  hal_data->do_init = toggle;
		
  //EC_WRITE_S32(&pd[hal_data->cmdvel_pdo_os], (int32_t)speed_raw);
  EC_WRITE_U32(&pd[hal_data->cmdvel_pdo_0+0],   0x10ff00);	
  EC_WRITE_U32(&pd[hal_data->cmdvel_pdo_1+6], 0xffef00ff);
}

