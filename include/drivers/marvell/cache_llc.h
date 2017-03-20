/*
 * ***************************************************************************
 * Copyright (C) 2015 Marvell International Ltd.
 * ***************************************************************************
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ***************************************************************************
 */

#ifndef _CACHE_LLC_H_
#define _CACHE_LLC_H_

void llc_cache_sync(void);
void llc_flush_all(void);
void llc_clean_all(void);
void llc_inv_all(void);
void llc_disable(void);
void llc_enable(int excl_mode);
int llc_is_exclusive(void);
void llc_save(void);
void llc_resume(void);

#endif /* _CACHE_LLC_H_ */

