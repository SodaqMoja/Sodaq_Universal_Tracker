/*
Copyright (c) 2015-18, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SODAQ_RN2483_INTERNAL_H_
#define SODAQ_RN2483_INTERNAL_H_

#define CRLF "\r\n"

#define STR_RESULT_OK "ok"
#define STR_RESULT_INVALID_PARAM "invalid_param"
#define STR_RESULT_MAC_ERROR "mac_err"
#define STR_RESULT_MAC_RX "mac_rx"
#define STR_RESULT_MAC_TX_OK "mac_tx_ok"

#define STR_RESULT_NOT_JOINED "not_joined"
#define STR_RESULT_NO_FREE_CHANNEL "no_free_ch"
#define STR_RESULT_SILENT "silent"
#define STR_RESULT_FRAME_COUNTER_ERROR "frame_counter_err_rejoin_needed"
#define STR_RESULT_BUSY "busy"
#define STR_RESULT_MAC_PAUSED "mac_paused"
#define STR_RESULT_INVALID_DATA_LEN "invalid_data_len"

#define STR_CMD_RESET "sys reset"
#define STR_DEVICE_TYPE_RN "RN"
#define STR_DEVICE_TYPE_RN2483 "RN2483"
#define STR_DEVICE_TYPE_RN2903 "RN2903"

#define STR_CMD_SET "mac set "
#define STR_RETRIES "retx "
#define STR_DEV_ADDR "devaddr "
#define STR_APP_SESSION_KEY "appskey "
#define STR_NETWORK_SESSION_KEY "nwkskey "
#define STR_DEV_EUI "deveui "
#define STR_APP_EUI "appeui "
#define STR_APP_KEY "appkey "
#define STR_ADR "adr "
#define STR_PWR_IDX "pwridx "
#define STR_DATARATE "dr "

#define STR_CMD_JOIN "mac join "
#define STR_OTAA "otaa"
#define STR_ABP "abp"
#define STR_ACCEPTED "accepted"

#define STR_CMD_MAC_TX "mac tx "
#define STR_CONFIRMED "cnf "
#define STR_UNCONFIRMED "uncnf "

#define STR_CMD_SLEEP "sys sleep 259200000" // 3 days
#define STR_CMD_GET_HWEUI "sys get hweui"
#define STR_CMD_SET_CHANNEL_STATUS "mac set ch status "

#endif
