#ifndef SHIM_H
#define SHIM_H

#define fabs(x)     		(((x)>0)?(x):-(x))
#define min(a,b)			((a<b)?a:b)

#define log_i       MPL_LOGI
#define log_e       MPL_LOGE

#ifdef __cplusplus
#define _EXTERN_ATTRIB extern "C"
#else
#define _EXTERN_ATTRIB
#endif

_EXTERN_ATTRIB int i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data);
_EXTERN_ATTRIB int i2c_read(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data);
_EXTERN_ATTRIB int reg_int_cb(struct int_param_s *int_param);
_EXTERN_ATTRIB void get_ms(unsigned long *count);
_EXTERN_ATTRIB void delay_ms(unsigned long ms);

#endif // SHIM_H
