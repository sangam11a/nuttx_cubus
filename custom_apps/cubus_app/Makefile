############################################################################
# apps/custom_apps/cubus_app/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# Cubus app built-in application info

PROGNAME  = $(CONFIG_CUSTOM_APPS_CUBUS_APP_PROGNAME)
PRIORITY  = $(CONFIG_CUSTOM_APPS_CUBUS_APP_PRIORITY)
STACKSIZE = $(CONFIG_CUSTOM_APPS_CUBUS_APP_STACKSIZE)
MODULE    = $(CONFIG_CUSTOM_APPS_CUBUS_APP)

# Cubus app Example

MAINSRC = cubus_app_main.c

CSRCS += adc.c
CSRCS += file_operations.c
CSRCS += common_functions.c
CRSCS += imu_mag.c
CRSCS += mission_operations.c
CRSCS += com_app_main.c

include $(APPDIR)/Application.mk