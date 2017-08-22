//
//    Copyright (C) 2014 Sascha Ittner <sascha.ittner@modusoft.de>

/*#############################################################################
#############################################################################*/

#ifndef _LCEC_AX5000_H_
#define _LCEC_AX5000_H_

#include "lcec.h"

#define LCEC_AX5000_VID LCEC_BECKHOFF_VID
#define LCEC_AX5206_PID 0x14566012

#define LCEC_AX5206_PDOS 8

int lcec_ax5000_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs);

#endif

