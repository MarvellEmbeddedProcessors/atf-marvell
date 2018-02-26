/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
 
#ifndef _MCI_H_
#define _MCI_H_

int mci_initialize(int mci_index);
void mci_turn_link_down(void);
void mci_turn_link_on(void);
int mci_get_link_status(void);

#endif /* _MCI_H_ */
