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

void llc_cache_sync(int);
void llc_flush_all(int);
void llc_clean_all(int);
void llc_inv_all(int);
void llc_disable(int);
void llc_enable(int, int excl_mode);
int llc_is_exclusive(int);
void llc_save(int);
void llc_resume(int);

#endif /* _CACHE_LLC_H_ */

