#ifndef __OCA72XXX_LOG_H__
#define __OCA72XXX_LOG_H__

#include <linux/kernel.h>


/********************************************
 *
 * print information control
 *
 *******************************************/
 
//#define OCS_DEBUG_LOG_ALL

#ifndef OCS_DEBUG_LOG_ALL
#define OCA_LOGI(fmt, ...)\
	pr_info("[OCSLOG] %s:" fmt "\n", __func__, ##__VA_ARGS__)

#define OCA_LOGD(fmt, ...)\
	pr_debug("[OCSLOG] %s:" fmt "\n", __func__, ##__VA_ARGS__)

#define OCA_LOGE(fmt, ...)\
	pr_err("[OCSLOG] %s:" fmt "\n", __func__, ##__VA_ARGS__)


#define OCA_DEV_LOGI(dev, fmt, ...)\
	pr_info("[OCSLOG] [%s]%s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define OCA_DEV_LOGD(dev, fmt, ...)\
	pr_debug("[OCSLOG] [%s]%s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define OCA_DEV_LOGE(dev, fmt, ...)\
	pr_err("[OCSLOG] [%s]%s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#else
#define OCA_LOGI(fmt, ...)\
	pr_info("[OCSLOG] %s:" fmt "\n", __func__, ##__VA_ARGS__)

#define OCA_LOGD(fmt, ...)\
	pr_info("[OCSLOG] %s:" fmt "\n", __func__, ##__VA_ARGS__)

#define OCA_LOGE(fmt, ...)\
	pr_err("[OCSLOG] %s:" fmt "\n", __func__, ##__VA_ARGS__)


#define OCA_DEV_LOGI(dev, fmt, ...)\
	pr_info("[OCSLOG] [%s]%s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define OCA_DEV_LOGD(dev, fmt, ...)\
	pr_info("[OCSLOG] [%s]%s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define OCA_DEV_LOGE(dev, fmt, ...)\
	pr_err("[OCSLOG] [%s]%s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#endif //DEBUG_LOG_ALL

#endif //__OCA72XXX_LOG_H__
