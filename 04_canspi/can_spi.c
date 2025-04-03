#include <linux/spi/spi.h>
#include <linux/can.h> // 为了 CAN_EFF_FLAG 和 CAN_RTR_FLAG
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

// spi通信使用的时钟频率
#define IPCAN_SPI_FREQ                  5000000

// 回显缓冲区(硬件不支持则软件需要设置大于等于1,硬件支持则可以设置为0)
#define TX_ECHO_SKB_MAX                 1

// can包的最大数据长度
#define CAN_FRAME_MAX_DATA_LEN	8
// 传输BUF的最大数据长度 4个16bit读寄存器操作 + 8个CAN数据?
#define SPI_TRANSFER_BUF_LEN	32

// 驱动设备的名称
#define DEVICE_NAME "bjhk_ipcan"

#define BJHK_IPCAN_OST_DELAY_MS     (5)


/* 定义IPUG132_CAN的寄存器 */
// 功能选择寄存器(r/w)
#define IPCAN_REG_FS                    0x00
// can使能寄存器(r/w)
#define IPCAN_REG_CE                    0x01
// 波特率预分配寄存器(r/w)
#define IPCAN_REG_BRP                   0x02
// 位时序寄存器(r/w)
#define IPCAN_REG_BT                    0x03
// 中断使能寄存器(r/w)
#define IPCAN_REG_IE                    0x04
// 中断状态寄存器(r)
#define IPCAN_REG_IS                    0x05
// 中断清楚寄存器(w)
#define IPCAN_REG_IC                    0x06
// 通信状态寄存器(r)
#define IPCAN_REG_CMOS                  0x07
// 错误状态寄存器(r/w1c(写1清除))
#define IPCAN_REG_ERRS                  0x08
// 错误计数器寄存器(r)
#define IPCAN_REG_ERRCNT                0x09
// 接收过滤器使能寄存器(r/w)
#define IPCAN_REG_AFE                   0x0a
// 接收过滤器标识符 1 掩码寄存器
#define IPCAN_REG_AFIDM1                0x0b
// 接收过滤器标识符 1 寄存器
#define IPCAN_REG_AFID1                 0x0c
// 接收过滤器标识符 2 掩码寄存器
#define IPCAN_REG_AFIDM2                0x0d
// 接收过滤器标识符 2 寄存器
#define IPCAN_REG_AFID2                 0x0e
// 接收过滤器标识符 3 掩码寄存器
#define IPCAN_REG_AFIDM3                0x0f
// 接收过滤器标识符 3 寄存器
#define IPCAN_REG_AFID3                 0x10
// 接收过滤器标识符 4 掩码寄存器
#define IPCAN_REG_AFIDM4                0x11
// 接收过滤器标识符 4 寄存器
#define IPCAN_REG_AFID4                 0x12
// 接收 fifo 标识符寄存器
#define IPCAN_REG_RXF_ID                0x13
// 接收 fifo 数据长度寄存器
#define IPCAN_REG_RXF_DL                0x14
// 接收 fifo 数据 1 寄存器
#define IPCAN_REG_RXF_DATA1             0x15
// 接收 fifo 数据 2 寄存器
#define IPCAN_REG_RXF_DATA2             0x16
// 发送 fifo 标识符寄存器
#define IPCAN_REG_TXF_ID                0x17
// 发送 fifo 数据长度寄存器
#define IPCAN_REG_TXF_DL                0x18
// 发送 fifo 数据 1 寄存器
#define IPCAN_REG_TXF_DATA1             0x19
// 发送 fifo 数据 2 寄存器
#define IPCAN_REG_TXF_DATA2             0x1a
// 发送高优先级缓存标识符寄存器
#define IPCAN_REG_TXHB_ID               0x1b
// 发送高优先级缓存数据长度寄存器
#define IPCAN_REG_TXHB_DL               0x1c
// 发送高优先级缓存数据 1 寄存器
#define IPCAN_REG_TXHB_DATA1            0x1d
// 发送高优先级缓存数据 2 寄存器
#define IPCAN_REG_TXHB_DATA2            0x1e


//一些寄存器值得定义
// 接收fifo非空中断
#define INT_RXFNEMPTY                       0x80
// 接收缓冲区溢出中断
#define INT_RXFWRFULL                       0x40
// 发送缓冲区溢出中断
#define INT_TXFFULL                         0x04
// 发送完成中断
#define INT_TXFINISH                    0x02
// 错误中断
#define INT_ERROR                       0x100
// busoff中断
#define INT_BUSOFF                      0x200
// errs:响应错误
#define ERR_FLAG_ACKERROR               0x10
// errs:位错误
#define ERR_FLAG_BITERROR               0x08
// errs:填充错误
#define ERR_FLAG_STUFFERROR             0x04
// errs:形式错误
#define ERR_FLAG_FORMERROR              0x02
// errs:crc错误
#define ERR_FLAG_CRCERROR               0x01


/* 一些寄存器配置值 */
// fs配置为回环模式
#define FS_VAL_LOOPBACK                 0x02
// fs配置为普通模式
#define FS_VAL_NORMAL                   0x00
// 使能ipcan
#define CE_VAL_ENABLE_IPCAN             0x02
// cmos通信状态为配置模式
#define CMOS_VAL_CFG                    0x01

// 写寄存器命令
#define IPCAN_CMD_WRITE(addr)  (((1 << 14) | ((addr & 0x3F) << 8)))
// 读寄存器命令
#define IPCAN_CMD_READ(addr)   (((2 << 14) | ((addr & 0x3F) << 8)))

#define CHECK_RET(ret)	{					\
							int _ret = ret;	\
							if(_ret != 0) {	\
								return -1;		\
							}				\
						}

/*  一些配置说明
    1. 功能选择(主要是配置fs)
        回环模式: fs.lb=1 fs.sleep=0
        休眠模式: fs.lb=0 fs.sleep=1
        通信模式: fs.lb=0 fs.sleep=0
    2. 配置can总线通信时序
        2.1 配置brp寄存器
        2.2 配置bt寄存器
    3. 接收过滤配置
        3.1 主要是配置 AFE AFIDM* AFID* 这几种寄存器
        3.2 挺麻烦的,配个damn
    4. 中断使能
        配置 IE 寄存器
    5. ip核使能
        将 ce.ce 置一
*/


// 供SPI的DMA使用
static int bjhk_ipcan_enable_dma; /* Enable SPI DMA. Default: 0 (Off) */
module_param(bjhk_ipcan_enable_dma, int, 0444);
MODULE_PARM_DESC(bjhk_ipcan_enable_dma, "ipcan: Enable SPI DMA. Default: 0 (Off)");


static const struct can_bittiming_const bjhk_ipcan_bittiming_const = {
	.name = DEVICE_NAME,
    /* 根据手册的 BT 寄存器说明,ph1即为tseg1(相位缓冲1)的取值范围(0~15 +1) 即最小值为1 */
	.tseg1_min = 1,
    /* 根据手册的 BT 寄存器说明,ph1即为tseg1的取值范围(0~15 +1) 即最大值为16 */
	.tseg1_max = 16,
    /* 与tseg1计算方法一样，看ph2寄存器，最小值1 */
	.tseg2_min = 1,
    /* 与tseg1计算方法一样，看ph2寄存器，最大值8 */
	.tseg2_max = 8,
    /* bt寄存器可得,最大值为4，最小值为1 */
	.sjw_max = 4,
    /* 根据 BRP 寄存器配置brp(波特率预分配器)相关的内容 */
    /* 最小值0+1 */
	.brp_min = 1,
    /* 最大值255+1 */
	.brp_max = 256,
    /* 递增步长 1 */
	.brp_inc = 1
};


struct bjhk_ipcan_priv {
	// 用于存储Linux SocketCAN 框架的CAN设备信息
	struct can_priv	   can;
	// Linux网络设备的抽象,socket can作为网络驱动设备,需要net_device进行数据收发
	struct net_device *net;
	// 用于存储spi设备线管的信息
	struct spi_device *spi;

	// 互斥锁,用于保护spi方位,避免多个线程同时访问can控制器,导致数据冲突
	struct mutex ipcan_lock; /* SPI device lock */

	// spi发送缓冲区
	u16 *spi_tx_buf;
	// spi接收缓冲区
	u16 *spi_rx_buf;
	// spi发送DMA地址
	dma_addr_t spi_tx_dma;
	// spi接收DMA地址
	dma_addr_t spi_rx_dma;
    // 是否使用dma的标志位
    u8 use_dma;

	// 发送队列
	struct sk_buff *tx_skb;
	// 发送队列的长度
	int tx_len;

	// 驱动使用的工作队列
	struct workqueue_struct *wq;
	// 用于处理can发送任务
	struct work_struct tx_work;
	// 用于处理can复位或者重启任务,如总线关闭后的重启
	struct work_struct restart_work;

	// 状态控制 如果出现严重错误，强制退出CAN控制器
	int force_quit;
    // 用于记录挂起/恢复过程的状态
	int after_suspend;
/* 
    表示挂起时设备是运行中的
    在 resume 时需要重新使能设备、恢复工作队列、重新启动 CAN 控制器等。
*/
#define AFTER_SUSPEND_UP 1
/* 
表示挂起时设备是未运行状态的 
恢复时不需要做任何初始化，resume 只需清除休眠位，其他交由之后的 open 处理
*/
#define AFTER_SUSPEND_DOWN 2
/* 
表示resume后要强制触发一次restart_work以重新初始化控制器 
用于某些异常恢复或状态同步场景，比如 do_set_mode() 中手动触发的情况
一般在bus-off后手动启动 
*/
#define AFTER_SUSPEND_RESTART 8
	// 用于记录是否开启错误重发功能
	int restart_tx;
};

// 清理can发送队列
static void bjhk_ipcan_clean(struct net_device *net)
{
    struct bjhk_ipcan_priv *priv = netdev_priv(net);

    // 如果还有数据,则记录失败
    if (priv->tx_skb || priv->tx_len)
        net->stats.tx_errors++;  // 记录 TX 失败次数

    if (priv->tx_skb)
        dev_kfree_skb(priv->tx_skb);  // 释放未发送的 skb

    if (priv->tx_len)
        can_free_echo_skb(priv->net, 0);  // 释放 CAN 发送回显缓存

    priv->tx_skb = NULL;
    priv->tx_len = 0;
}

/*
IPCAN通用SPI传输函数,传输的数据是u16类型的
*/
static int bjhk_ipcan_spi_trans(struct spi_device *spi, int len)
{
    struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);
    struct spi_transfer t = {
        .tx_buf = priv->spi_tx_buf,
        .rx_buf = priv->spi_rx_buf,
        .len = len,
        .cs_change = 0,  // 传输结束后 CS 保持不变
    };
    struct spi_message m;
    int ret;

    spi_message_init(&m);

    if (bjhk_ipcan_enable_dma) {  // 如果支持 DMA
        t.tx_dma = priv->spi_tx_dma;
        t.rx_dma = priv->spi_rx_dma;
        m.is_dma_mapped = 1;
    }

    spi_message_add_tail(&t, &m);

    ret = spi_sync(spi, &m);
    if (ret)
        dev_err(&spi->dev, "IPCAN : SPI transfer failed: ret = %d\n", ret);
    return ret;
}

/*
IPCAN读寄存器
*/
static int ipcan_read_register(struct spi_device *spi, u16 addr, u32 *value)
{
    struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);
    int ret = 0;

    priv->spi_tx_buf[0] = IPCAN_CMD_READ(addr); // 读命令 + 地址
    priv->spi_tx_buf[1] = 0x00000000; // Dummy 数据
    priv->spi_tx_buf[2] = 0x00000000; // 用于接收寄存器高 16bit
    priv->spi_tx_buf[3] = 0x00000000; // 用于接收寄存器低 16bit

    ret = bjhk_ipcan_spi_trans(spi, 4*sizeof(u16));
    if (ret < 0)
        return ret;

    *value = ((u32)priv->spi_rx_buf[2] << 16) | priv->spi_rx_buf[3]; // 组合高低 16bit

    return 0;
}

/*
IPCAN写寄存器
*/
static int ipcan_write_register(struct spi_device *spi, u16 addr, u32 value)
{
    struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);

    priv->spi_tx_buf[0] = IPCAN_CMD_WRITE(addr); // 写命令 + 地址
    priv->spi_tx_buf[1] = (value >> 16) & 0xFFFF; // 高 16bit
    priv->spi_tx_buf[2] = value & 0xFFFF; // 低 16bit

    return bjhk_ipcan_spi_trans(spi, 3*sizeof(u16));
}

// ipcan清空所有寄存器
static int bjhk_ipcan_clear_all_regs(struct spi_device *spi)
{
    int ret = -1;
	ret = ipcan_write_register(spi, IPCAN_REG_CE, 1); 			// 1.CAN使能寄存器(优先配置这个是因为有的寄存器需要CE位为0才能配置)
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_CE success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_FS, 0); 			// 2.功能选择寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_FS success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_BRP, 0); 			// 3.波特率预分频寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_BRP success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_BT, 0); 			// 4.位时序寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_BT success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_IE, 0); 			// 5.中断使能寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_IE success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_IC, 0xFFF); 		// 6.中断清除寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_IC success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_ERRS, 0x1f); 		// 7.错误状态寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_ERRS success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFE, 0); 			// 8.接收过滤器使能寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFE success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFIDM1, 0); 		// 9.接收过滤器标识符 1 掩码寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFIDM1 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFID1, 0); 		// 10.接收过滤器标识符 1 寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFID1 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFIDM2, 0); 		// 11.接收过滤器标识符 2 掩码寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFIDM2 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFID2, 0); 		// 12.接收过滤器标识符 2 寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFID2 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFIDM3, 0); 		// 13.接收过滤器标识符 3 掩码寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFIDM3 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFID3, 0); 		// 14.接收过滤器标识符 3 寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFID3 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFIDM4, 0); 		// 15.接收过滤器标识符 4 掩码寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFIDM4 success!\r\n");
	ret = ipcan_write_register(spi, IPCAN_REG_AFID4, 0); 		// 16.接收过滤器标识符 4 寄存器
	CHECK_RET(ret);
	dev_info(&spi->dev, "clear IPCAN_REG_AFID4 success!\r\n");

	return 0;
}

/*
读取错误寄存器中错误值
*/
static int bjhk_ipcan_get_berr_counter(const struct net_device *dev, struct can_berr_counter *bec)
{
    struct bjhk_ipcan_priv *priv = netdev_priv(dev);
    u32 errcnt;
    int ret;

    /* 确保错误情况下不会返回未初始化的数据 */
    bec->txerr = 0;
    bec->rxerr = 0;

    /* 读取 ERR_CNT 寄存器 */
    ret = ipcan_read_register(priv->spi, IPCAN_REG_ERRCNT, &errcnt);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to read ERR_CNT register, ret = %d\n", ret);
        return ret;
    }

    /* 解析错误计数 */
    bec->txerr = errcnt & 0xFF;        // 低 8 位是 TEC（发送错误计数）
    bec->rxerr = (errcnt >> 8) & 0xFF; // 高 8 位是 REC（接收错误计数）

    return 0;
}

/*
用于合成需要存放到txf-id寄存器的值
*/
// 将扩展帧ID存入TXF-ID寄存器
u32 get_txfid_by_id(u32 can_id)
{
	u32 data = 0;
	u8 ide = (can_id & CAN_EFF_FLAG) ? 1 : 0;        // 是否扩展帧
	u8 rtr = (can_id & CAN_RTR_FLAG) ? 1 : 0;        // 是否远程帧

	if (ide) {
		// 扩展帧
		u16 bid = (can_id >> 18) & 0x7FF;            // 标准帧部分
		u32 eid = can_id & 0x3FFFF;             // 低18位扩展ID

		data |= (bid & 0x7FF) << 21;                      // BID[31:21]
		data |= 0 << 20;                                  // SRR = 0（保留）
		data |= 1 << 19;                                  // IDE = 1 扩展帧
		data |= (eid & 0x3FFFF) << 1;                     // EID[18:1]
		data |= rtr;                                      // RTR
	} else {
		// 标准帧
		u16 sid = can_id & CAN_SFF_MASK;

		data |= (sid & 0x7FF) << 21;                      // BID[31:21]
		data |= rtr << 20;                                // SRR/RTR
		data |= 0 << 19;                                  // IDE = 0 标准帧
		// EID 和 bit0 不用设置
	}

	return data;
}

/*
这个函数用于将CAN数据输入到ipcan的发送缓冲区(写入寄存器)
*/
static void bjhk_ipcan_hw_tx_frame(struct spi_device *spi, u32 txfid,
				u32 dlc, u32 data1, u32 data2)
{
	struct bjhk_ipcan_priv *priv = NULL;
    priv = spi_get_drvdata(spi);

	ipcan_write_register(spi, IPCAN_REG_TXF_ID, txfid);
    ipcan_write_register(spi, IPCAN_REG_TXF_DL, dlc);
    ipcan_write_register(spi, IPCAN_REG_TXF_DATA1, data1);
    ipcan_write_register(spi, IPCAN_REG_TXF_DATA2, data2);
}

/*
被外部调用的数据发送函数
*/
static void bjhk_ipcan_hw_tx(struct spi_device *spi, struct can_frame *frame)
{
	struct bjhk_ipcan_priv *priv = NULL;
	u32 txfid = 0;
    u32 dlc = 0;
    u32 data1 = 0;
    u32 data2 = 0;

    priv = spi_get_drvdata(spi);

    // 处理id的内容
    txfid = get_txfid_by_id(frame->can_id);

    // 处理dlc
    dlc = frame->can_dlc;
    if (dlc > 8) {
        dlc = 8;
    }
    dlc = dlc<<28;

    // 处理data1
    // 将0~3四个字节存放到data1中
	data1 |= frame->data[0] << 24;
	data1 |= frame->data[1] << 16;
	data1 |= frame->data[2] << 8;
	data1 |= frame->data[3] << 0;
    // 处理data2
    // 将4~7四个字节存放到data2中
	data2 |= frame->data[4] << 24;
	data2 |= frame->data[5] << 16;
	data2 |= frame->data[6] << 8;
	data2 |= frame->data[7] << 0;

	// 将数据发送出去
    bjhk_ipcan_hw_tx_frame(spi, txfid, dlc, data1, data2);
}

/*
从ipcan寄存器中读取数据
*/
static void bjhk_ipcan_hw_rx_frame(struct spi_device *spi, u32 *rxf_id, u32 *rxf_dl, u32 *data1, u32 *data2)
{
	struct bjhk_ipcan_priv *priv;
    priv = spi_get_drvdata(spi);

	// 读取 rxf-id 寄存器
    ipcan_read_register(spi, IPCAN_REG_RXF_ID, rxf_id);
    // 读取 rxf-dl 寄存器
    ipcan_read_register(spi, IPCAN_REG_RXF_DL, rxf_dl);
    // 读取 rxf-data1 寄存器
    ipcan_read_register(spi, IPCAN_REG_RXF_DATA1, data1);
    // 读取 rxf-data2 寄存器
    ipcan_read_register(spi, IPCAN_REG_RXF_DATA2, data2);
}

/*
ipcan的数据接收函数
*/
static void bjhk_ipcan_hw_rx(struct spi_device *spi)
{
	struct bjhk_ipcan_priv *priv;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 rxf_id = 0;
    u32 rxf_dl = 0;
    u32 data1 = 0;
    u32 data2 = 0;

    u8 ide = 0;
    u8 srr_rtr = 0;
    u8 rtr = 0;
    u16 bid = 0;
    u32 eid = 0;

    int i= 0;

    priv = spi_get_drvdata(spi);

	// 申请缓冲区
	skb = alloc_can_skb(priv->net, &frame);
	if (!skb) {
		dev_err(&spi->dev, "ipcan: cannot allocate RX skb\n");
		priv->net->stats.rx_dropped++;
		return;
	}

	// 接收CAN数据
	bjhk_ipcan_hw_rx_frame(spi, &rxf_id, &rxf_dl, &data1, &data2);
    ide = (rxf_id >> 19) & 0x1;         // IDE: 扩展帧标志位
    srr_rtr = (rxf_id >> 20) & 0x1;     // SRR/RTR: 标准帧或扩展帧的远程标志
    rtr = (rxf_id >> 0) & 0x1;          // RTR: 远程请求帧（扩展帧时有效）
    bid = (rxf_id >> 21) & 0x7FF;      // 标准帧 ID（11 位）
    eid = (rxf_id >> 1) & 0x3FFFF;     // 扩展帧 ID（18 位）
    
	// 如果是扩展帧
	if (ide) {
		/* Extended ID format */
        // 扩展帧：bit20 是 SRR（保留），bit0 是 RTR
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |= (eid | ((uint32_t)bid << 18));  // 合成完整 29 位 ID
		/* Remote transmission request */
		if (rtr)                                // 判断是否远程帧
            frame->can_id |= CAN_RTR_FLAG;
	} else {
		// 标准帧：bit20 是 RTR
		/* Standard ID format */
        frame->can_id = bid;                    // 直接使用标准帧 ID
        if (srr_rtr)                            // SRR/RTR位作为标准帧远程标志
            frame->can_id |= CAN_RTR_FLAG;
	}
	/* Data length */
	// 获取数据长度
	frame->can_dlc = get_can_dlc((rxf_dl >> 28) & 0xF);

    // 处理can.data
    // 拆分 data1 和 data2 为 frame->data[0~7]
    for (i = 0; i < frame->can_dlc && i < 4; i++) {
        frame->data[i] = (data1 >> (i * 8)) & 0xFF;
    }

    for (i = 0; i < frame->can_dlc - 4 && i < 4; i++) {
        frame->data[i + 4] = (data2 >> (i * 8)) & 0xFF;
    }

	// CAN网络接收计数器加一
	priv->net->stats.rx_packets++;
	// can网络接收字节加dlc
	priv->net->stats.rx_bytes += frame->can_dlc;

	// 配置can的led,没有不需要配置
	// can_led_event(priv->net, CAN_LED_EVENT_RX);

	// 上报接收到的can数据给网络帧
	netif_rx_ni(skb);
}

/*
IPCAN进入休眠模式
*/
static void bjhk_ipcan_hw_sleep(struct spi_device *spi)
{
	ipcan_write_register(spi, IPCAN_REG_FS, (1<<0));
}

/*
发送入口函数
*/
static netdev_tx_t bjhk_ipcan_hard_start_xmit(struct sk_buff *skb,
					   struct net_device *net)
{
	struct bjhk_ipcan_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	// 代表发送繁忙 防止覆盖未发送完成的数据
    if (priv->tx_skb || priv->tx_len) {
		dev_warn(&spi->dev, "ipcan : hard_xmit called while tx busy\n");
		return NETDEV_TX_BUSY;
	}

	// 检测数据包合法性,如果是非法包则丢弃(如dlc大于8)
    if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	// 通知内核发送队列满了,暂时不允许调用xmit
    netif_stop_queue(net);
    // 将skb缓存到私有变量中
	priv->tx_skb = skb;
    // 将发送任务加入tx工作队列
	queue_work(priv->wq, &priv->tx_work);

	// 表示已经接收该帧,稍后会一步发送数据(并不是发送成功)
    return NETDEV_TX_OK;
}

/*
    ipcan设置模式的函数
*/
static int bjhk_ipcan_do_set_mode(struct net_device *net, enum can_mode mode)
{
    struct bjhk_ipcan_priv *priv;
    priv = netdev_priv(net);

    switch (mode) {
    case CAN_MODE_START:
        bjhk_ipcan_clean(net);  // 清除错误状态（如果需要）
        priv->can.state = CAN_STATE_ERROR_ACTIVE; // 设为正常工作状态
        priv->restart_tx = 1;
        if (priv->can.restart_ms == 0)
			priv->after_suspend = AFTER_SUSPEND_RESTART;
        queue_work(priv->wq, &priv->restart_work); // 触发 CAN 重新启动
        break;
    default:
        return -EOPNOTSUPP;
    }

    return 0;
}

/*
配置CAN进入正常的通讯模式
*/
static int bjhk_ipcan_set_normal_mode(struct spi_device *spi)
{
	struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);
	unsigned long timeout;
    u32 ie_val = 0;
    u32 val = 1;

	/* Enable interrupts */
	/*
    以下中断需要使能
        TXFINISH : 同时完成中断
        RXFWRFULL : 接收FIFO满中断
        RXFNEMPTY : 接收FIFO非空中断
        ERROR : 错误中断
        BUSOFF : busoff中断
    */
    ie_val |= INT_TXFINISH;
    ie_val |= INT_RXFWRFULL;
    ie_val |= INT_RXFNEMPTY;
    ie_val |= INT_ERROR;
    ie_val |= INT_BUSOFF;
    // 写can寄存器
    ipcan_write_register(spi, IPCAN_REG_IE, ie_val);

    // 根据CAN模式配置
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* 配置回环模式 */
		ipcan_write_register(spi, IPCAN_REG_FS, FS_VAL_LOOPBACK);
        // 写ce
        ipcan_write_register(spi, IPCAN_REG_CE, CE_VAL_ENABLE_IPCAN);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* 硬件没有监听模式 */
		dev_warn(&spi->dev, "Listen-only mode not supported on IPCAN\n");
	} else {
		/* 设置为正常模式 */
        // 写fs为
        ipcan_write_register(spi, IPCAN_REG_FS, FS_VAL_NORMAL);

        // 写ce
        ipcan_write_register(spi, IPCAN_REG_CE, CE_VAL_ENABLE_IPCAN);

		/* 读cmos寄存器,等待配置成功 */
		timeout = jiffies + HZ;
        while(val & CMOS_VAL_CFG) {
            ipcan_read_register(spi, IPCAN_REG_CMOS, &val);
            if (time_after(jiffies, timeout)) {
				dev_err(&spi->dev, "IPCAN didn't"
					" enter in normal mode\n");
				return -EBUSY;
			}
            cpu_relax(); // 防止cpu空转
            schedule(); // 防止cpu空转
        }
	}
	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

/*
配置波特率
*/
static int bjhk_ipcan_do_set_bittiming(struct net_device *net)
{
	struct bjhk_ipcan_priv *priv = netdev_priv(net);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct spi_device *spi = priv->spi;
    u32 bt_val = 0;
    u32 brp_val = 0;

	// 配置bt寄存器 == sjw  tseg1(prop_seg+prop_seg1)  tseg2(prop_seg2)
    // 构造 BT 寄存器值
	bt_val |= ((bt->sjw - 1) & 0x3) << 7;         // SJW [8:7]
	bt_val |= ((bt->phase_seg2 - 1) & 0x7) << 4;  // PH2 [6:4]
	bt_val |= ((bt->phase_seg1 + bt->prop_seg - 1) & 0xF); // PH1 [3:0]

	// 配置brp
    brp_val |= (bt->brp-1)&0xff;

    // 将配置写入寄存器
    ipcan_write_register(spi, IPCAN_REG_BT, bt_val);
    ipcan_write_register(spi, IPCAN_REG_BRP, brp_val);

    bt_val = 0;
    brp_val = 0;
    ipcan_read_register(spi, IPCAN_REG_BT, &bt_val);
    ipcan_read_register(spi, IPCAN_REG_BRP, &brp_val);

	dev_dbg(&spi->dev, "IPCAN bjhk_ipcan_do_set_bittiming: reg_bt:0x%02x reg_brp:0x%02x \n",bt_val,brp_val);

	return 0;
}

/*
配置规则模式
*/
static int bjhk_ipcan_setup(struct net_device *net, struct spi_device *spi)
{
    // 因为不需要配置接收缓冲区,所以只需要配置波特率即可
	return bjhk_ipcan_do_set_bittiming(net);
}

// 向ipcan发送复位指令,确保进入复位模式
static int bjhk_ipcan_hw_reset(struct spi_device *spi)
{
    struct bjhk_ipcan_priv *priv;
    unsigned long timeout;
    int ret;
    u32 val = 0;

    priv = spi_get_drvdata(spi);

    // 清空所有寄存器
    ret = bjhk_ipcan_clear_all_regs(spi);

    // 发送复位指令
    ret = ipcan_write_register(spi, IPCAN_REG_CE, 0x1);
	if(ret != 0) {
		dev_err(&spi->dev,
            "error! enable_ipcan  IPCAN_REG_CE failed!\n");
        return ret;
	}

	// 轮询查询是否进入配置模式
    timeout = jiffies + HZ; // 1s的超时时间
    while ( (val & 0x1) != 1) {
		ipcan_read_register(spi, IPCAN_REG_CMOS, &val);
        usleep_range(BJHK_IPCAN_OST_DELAY_MS * 1000,
			     BJHK_IPCAN_OST_DELAY_MS * 1000 * 2);

		if (time_after(jiffies, timeout)) {
			dev_err(&spi->dev,
				"hjhk_ipcan didn't enter in conf mode after reset\n");
            // 超时返回失败
			return -EBUSY;
		}
	}
    
    // 打印hw_reset成功
    dev_info(&spi->dev, "bjhk_ipcan_hw_reset: success!\n");
    
    return 0;
}

/*
    检测ipcan是否存在
    验证spi通信是否正常
    确认ipcan是否进入配置模式
*/
static int bjhk_ipcan_hw_probe(struct spi_device *spi)
{
	u32 ctrl;
	int ret;

    // 确认硬件是否存在
	ret = bjhk_ipcan_hw_reset(spi);
	if (ret)
		return ret;

    // 读取cmos寄存器，看是否进入配置模式
	ret = ipcan_read_register(spi, IPCAN_REG_CMOS, &ctrl);
    if (ret) {
        dev_info(&spi->dev, "bjhk_ipcan_hw_probe : failed to read IPCAN_REG_CMOS!\r\n");
        return ret;
    }

	dev_dbg(&spi->dev, "IPCAN_REG_CMOS 0x%04x\n", ctrl);

	/* Check for power up default value */
	if ((ctrl & 0x1) != 1) {
        dev_info(&spi->dev, "bjhk_ipcan_hw_probe : ipcan not in cfg mode!\r\n");
        return -ENODEV;
    }

	return 0;
}

/*
ipcan退出回调函数
*/
static int bjhk_ipcan_stop(struct net_device *net)
{
	struct bjhk_ipcan_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	// 关闭can网络设备,释放资源
    close_candev(net);

	// 设置允许状态标识位
    priv->force_quit = 1;
    // 释放中断
	free_irq(spi->irq, priv);
    // 销毁工作队列
	destroy_workqueue(priv->wq);
	priv->wq = NULL;

	// 加锁访问寄存器
    mutex_lock(&priv->ipcan_lock);

	/* Disable and clear pending interrupts */
    // 禁用中断
	ipcan_write_register(spi, IPCAN_REG_IE, 0x00);
    // 清除中断标识位
	ipcan_write_register(spi, IPCAN_REG_IC, 0xFFF);
    // 复位
	ipcan_write_register(spi, IPCAN_REG_CE, 0x01);
	bjhk_ipcan_clean(net);

	bjhk_ipcan_hw_sleep(spi);

	priv->can.state = CAN_STATE_STOPPED;

	mutex_unlock(&priv->ipcan_lock);

	return 0;
}

/*
    错误处理函数
*/
static void bjhk_ipcan_error_skb(struct net_device *net, int can_id, int dlc, u8 *data)
{
	struct sk_buff *skb;
	struct can_frame *frame;
    int i;

	skb = alloc_can_err_skb(net, &frame);
	if (skb) {
		frame->can_id |= can_id;
        if (dlc > 8)
            dlc = 8;
        if (dlc < 0)
            dlc = 1;
        for (i=0; i<dlc; i++) {
            frame->data[i] = data[i];
        }
		
		netif_rx_ni(skb);
	} else {
		netdev_err(net, "cannot allocate error skb\n");
	}
}

/*
发送工作队列服务函数
*/
static void bjhk_ipcan_tx_work_handler(struct work_struct *ws)
{
	struct bjhk_ipcan_priv *priv = container_of(ws, struct bjhk_ipcan_priv,
						 tx_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	struct can_frame *frame;

	mutex_lock(&priv->ipcan_lock);
	if (priv->tx_skb) {
		if (priv->can.state == CAN_STATE_BUS_OFF) {
			bjhk_ipcan_clean(net);
		} else {
			frame = (struct can_frame *)priv->tx_skb->data;

			if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
				frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
			bjhk_ipcan_hw_tx(spi, frame);
			priv->tx_len = 1 + frame->can_dlc;
			can_put_echo_skb(priv->tx_skb, net, 0);
			priv->tx_skb = NULL;
		}
	}
	mutex_unlock(&priv->ipcan_lock);
}

static void bjhk_ipcan_restart_work_handler(struct work_struct *ws)
{
	struct bjhk_ipcan_priv *priv = container_of(ws, struct bjhk_ipcan_priv,
						 restart_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
    u8 data = 0;

	mutex_lock(&priv->ipcan_lock);
	if (priv->after_suspend) {
		bjhk_ipcan_hw_reset(spi);
		bjhk_ipcan_setup(net, spi);
		priv->force_quit = 0;
		if (priv->after_suspend & AFTER_SUSPEND_RESTART) {
			bjhk_ipcan_set_normal_mode(spi);
		} else if (priv->after_suspend & AFTER_SUSPEND_UP) {
			netif_device_attach(net);
			bjhk_ipcan_clean(net);
			bjhk_ipcan_set_normal_mode(spi);
			netif_wake_queue(net);
		} else {
			bjhk_ipcan_hw_sleep(spi);
		}
		priv->after_suspend = 0;
	}

	if (priv->restart_tx) {
        // 清空restart标识位
		priv->restart_tx = 0;
        // 清空socket缓存的回显skb
		bjhk_ipcan_clean(net);
        // 唤醒网络队列
		netif_wake_queue(net);
        // 上报重启错误帧
        data = 0;
		bjhk_ipcan_error_skb(net, CAN_ERR_RESTARTED, 1, &data);
	}
	mutex_unlock(&priv->ipcan_lock);
}

/*
中断服务函数
*/
static irqreturn_t bjhk_ipcan_interrupt_handler(int irq, void *dev_id)
{
    struct bjhk_ipcan_priv *priv = dev_id; // 驱动控制块
    struct spi_device *spi = priv->spi; // spi的设备控制块
	struct net_device *net = priv->net; // 网络设备控制块

    // 加锁
	mutex_lock(&priv->ipcan_lock);
	// 只有CAN处于运行状态才进行下面的操作
    while (!priv->force_quit) {
        /*
            new_state:用于表示can的状态 定义在 <linux/can/netlink.h>
                    一般有以下四个状态
            CAN_STATE_ERROR_ACTIVE：错误主动状态

            CAN_STATE_ERROR_WARNING：错误警告状态

            CAN_STATE_ERROR_PASSIVE：错误被动状态

            CAN_STATE_BUS_OFF：总线关闭状态
        */
        enum can_state new_state;
		u32 intf = 0; // intf:中断类型
        u32 eflag = 0; // eflag:错误标识
        u32 err_count = 0; // 错误计数器
        u8 tx_err_count = 0; // 发送错误计数器
        u8 rx_err_count = 0; // 接收错误计数器
		u32 clear_intf = 0; // 记录需要清除的中断位
		int can_id = 0; // can_id:错误帧的标识ID  data1:错误帧的数据字段
        u8 data1 = 0;

        // 读取中断寄存器
        ipcan_read_register(spi, IPCAN_REG_IS, &intf);
        // 读取错误状态寄存器
        ipcan_read_register(spi, IPCAN_REG_ERRS, &eflag);
        // 读取错误计数器寄存器
        ipcan_read_register(spi, IPCAN_REG_ERRCNT, &err_count);

        // 判断中断类型
        /* 如果是接收中断
        与接收相关的中断有很多: 
            RXFINISH: 接收完成中断
            RXFRDEMPTY: 接收fifo读空中断
            INT_RXFWRFULL: 接收fifo写满中断
            INT_RXFNEMPTY: 接收fifo非空中断
            这么多中断，这里就用 INT_RXFNEMPTY 这个非空中断
        */
        if(intf & INT_RXFNEMPTY) {
            // 接收数据
            bjhk_ipcan_hw_rx(spi);
            // 标记该中断标识位需要清空
            clear_intf |= INT_RXFNEMPTY;
        }

        // 发送相关得中断或者错误相关得中断需要处理
        if(intf & (INT_TXFINISH | INT_ERROR | INT_BUSOFF)) {
            clear_intf |= (INT_TXFINISH | INT_ERROR | INT_BUSOFF);
        }
        // 清除中断标识位
        if (clear_intf)
			ipcan_write_register(spi, IPCAN_REG_IC, clear_intf);

        // 读取错误标识位，然后在这里清除,mcp251x是这么做的
        if (eflag) {
            // 先清除标识位,mcp251x是这么做的
            ipcan_write_register(spi, IPCAN_REG_ERRS, eflag);
        }

        /*
            分析错误状态并生成错误帧
            主要是设置这四种状态:
                CAN_STATE_BUS_OFF : can busoff 状态
                CAN_STATE_ERROR_PASSIVE : 警告状态
                CAN_STATE_ERROR_WARNING : 被动错误状态
                CAN_STATE_ERROR_ACTIVE : 正常状态
        */
        // 计算发送错误
        tx_err_count = err_count & 0xff;
        // 计算接收错误
        rx_err_count = ((err_count & 0xff00)>>8);
        if(intf&INT_BUSOFF) {
            // busoff
            new_state = CAN_STATE_BUS_OFF;
			can_id |= CAN_ERR_BUSOFF;
        }
        else if(tx_err_count >= 128) {
            // 发送错误(tx_count>=128)
            new_state = CAN_STATE_ERROR_PASSIVE;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_TX_PASSIVE;
        } else if(rx_err_count >= 128) {
            // 接收错误(rx_count>=128)
            new_state = CAN_STATE_ERROR_PASSIVE;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_PASSIVE;
        } else if((tx_err_count < 128) && (tx_err_count >= 96)) {
            // 发送错误(96<=tx_count<128)
            new_state = CAN_STATE_ERROR_WARNING;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_TX_WARNING;
        } else if((rx_err_count < 128) && (rx_err_count >= 96)) {
            // 接收错误(96<=rx_count<128)
			new_state = CAN_STATE_ERROR_WARNING;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_WARNING;
        } else {
            // 正常状态(rx_count<96 && tx_count<96)
			new_state = CAN_STATE_ERROR_ACTIVE;
        }

        // 统计错误状态切换次数 配置can的错误状态计数器
        switch (priv->can.state) {
            case CAN_STATE_ERROR_ACTIVE:
                if (new_state >= CAN_STATE_ERROR_WARNING &&
                    new_state <= CAN_STATE_BUS_OFF)
                    priv->can.can_stats.error_warning++;
            case CAN_STATE_ERROR_WARNING:	/* fallthrough 好像是一种高级用法!*/
                if (new_state >= CAN_STATE_ERROR_PASSIVE &&
                    new_state <= CAN_STATE_BUS_OFF)
                    priv->can.can_stats.error_passive++;
                break;
            default:
                break;
		}
        priv->can.state = new_state;

        // 处理溢出故障
        // 确认是否处于错误状态，是则发送错误包
        if(can_id) {
            // 接收fifo溢出
            if(intf & INT_RXFWRFULL) {
                net->stats.rx_over_errors++;
                net->stats.rx_errors++;
                can_id |= CAN_ERR_CRTL;
                data1 |= CAN_ERR_CRTL_RX_OVERFLOW;
            }
            // 这里只做了一个overflow的错误,其他的如ack错误、填充错误这些故障没有在这里上报
            bjhk_ipcan_error_skb(net, can_id, 1, &data1);
        }

        // 如果总线关闭则判断是否字段恢复
        if (priv->can.state == CAN_STATE_BUS_OFF) {
			if (priv->can.restart_ms == 0) {
				priv->force_quit = 1;
				priv->can.can_stats.bus_off++;
				can_bus_off(net);
				bjhk_ipcan_hw_sleep(spi);
				break;
			}
		}

        if (intf == 0)
			break;
        

        // 发送完成中断处理
        if (intf & INT_TXFINISH) {
			net->stats.tx_packets++;
			net->stats.tx_bytes += priv->tx_len - 1;
			// can_led_event(net, CAN_LED_EVENT_TX);
			if (priv->tx_len) {
				can_get_echo_skb(net, 0);
				priv->tx_len = 0;
			}
			netif_wake_queue(net);
		}

    }

    mutex_unlock(&priv->ipcan_lock);
    return IRQ_HANDLED;
}

// 打开can设备时调用
static int bjhk_ipcan_open(struct net_device *net)
{
    // 获取私有数据域
    struct bjhk_ipcan_priv *priv = netdev_priv(net);
    // 获取spi设备块
	struct spi_device *spi = priv->spi;
    // 设置中断触发标识 一次性触发中断和下降沿触发中断
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_RISING;
    // 用于获取返回值
	int ret = 0;

    // 打开CAN设备
    ret = open_candev(net);
	if (ret) {
		dev_err(&spi->dev, "IPCAN : unable to set initial baudrate!\n");
		return ret;
	}

    // 加锁，防止并发访问
    mutex_lock(&priv->ipcan_lock);

    // 清空TX相关变量
    // 设置运行状态标志位位正常状态
    priv->force_quit = 0;
    // 清空发送缓冲区,防止残留数据影响CAN发送
	priv->tx_skb = NULL;
    // 清空发送缓冲区长度,使任务从0开始
	priv->tx_len = 0;

    // 申请中断
    // 直接使用设备树中spi->irq配置的中断  bjhk_ipcan_interrupt_handler:中断处理函数
    ret = request_threaded_irq(spi->irq, NULL, bjhk_ipcan_interrupt_handler,
				   flags, DEVICE_NAME, priv);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
		goto out_close;
	}

    // 创建工作队列
    /* bjhk_ipcan_wq : 工作队列的名称,在调试时可以看到 */
    /* WQ_FREEZABLE:运行工作队列在系统挂起时被冻结 防止在系统进入休眠模式后继续执行CAN导致出问题*/
    /* WQ_MEM_RECLAIM:保证工作队列的任务可以在内存回收时继续执行 防止CAN设备因为内存压力无法运行*/
    /* 第三个参数0: 使用默认的最大并发数,让系统自动调整workqueue的调度策略 */
	priv->wq = alloc_workqueue("bjhk_ipcan_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	if (!priv->wq) {
		ret = -ENOMEM;
		goto out_clean;
	}
    // 初始化 tx_work 工作任务
	INIT_WORK(&priv->tx_work, bjhk_ipcan_tx_work_handler);
    // 初始化 restart_work 工作任务
	INIT_WORK(&priv->restart_work, bjhk_ipcan_restart_work_handler);

    // 复位IPCAN
	ret = bjhk_ipcan_hw_reset(spi);
	if (ret)
		goto out_free_wq;
	// 设置波特率、接收缓冲区、过滤模式等
	ret = bjhk_ipcan_setup(net, spi);
	if (ret)
		goto out_free_wq;
    // 使能中断，进入mormal模式
	ret = bjhk_ipcan_set_normal_mode(spi);
	if (ret)
		goto out_free_wq;

    netif_wake_queue(net);
	mutex_unlock(&priv->ipcan_lock);

	return 0;

out_free_wq:
	destroy_workqueue(priv->wq);
out_clean:
	free_irq(spi->irq, priv);
	bjhk_ipcan_hw_sleep(spi);
out_close:
	close_candev(net);
	mutex_unlock(&priv->ipcan_lock);
	return ret;
}

static const struct net_device_ops bjhk_ipcan_netdev_ops = {
    /* 负责启动CAN设备 */
	.ndo_open = bjhk_ipcan_open,
    /* 负责停止CAN设备 */
	.ndo_stop = bjhk_ipcan_stop,
    /* 负责发送CAN数据 */
	.ndo_start_xmit = bjhk_ipcan_hard_start_xmit,
    // 虽然不支持fd,但是需要加上这个做合法性检验
    .ndo_change_mtu = can_change_mtu,
};

// 设备树匹配表
static const struct of_device_id bjhk_ipcan_of_match[] = {
	{
		.compatible	= "bjhk,ipcan",
	},
    { }
};
MODULE_DEVICE_TABLE(of, bjhk_ipcan_of_match);

// 用于id匹配表
static const struct spi_device_id bjhk_ipcan_id_table[] = {
	{
		.name		= "ipcan0",
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, bjhk_ipcan_id_table);

/*
    probe函数
    需要实现 设备初始化、分配资源、注册CAN设备等
*/
static int bjhk_ipcan_can_probe(struct spi_device *spi)
{
    const struct of_device_id *match;
    struct net_device *net;
    struct bjhk_ipcan_priv *priv;
    int ret;
    int freq; // can的时钟配置

    /* 通过设备树匹配表获取 CAN 设备类型 */
    match = of_match_device(bjhk_ipcan_of_match, &spi->dev);
    if (!match) {
        dev_err(&spi->dev, "Failed to match device\n");
        return -ENODEV;
    }

    /* -------------------一些通用初始化-------------------*/
    // 打印初始化信息
    dev_info(&spi->dev, "bjhk_ipcan CAN driver probing...\n");

    /* TODO: 申请并初始化 CAN 设备、分配 buffer、注册 net_device */
    /* -------------------申请资源-------------------*/
    // 申请can dev
	net = alloc_candev(sizeof(struct bjhk_ipcan_priv), TX_ECHO_SKB_MAX);
	if (!net) {
        return -ENOMEM;
    }
    
    net->netdev_ops = &bjhk_ipcan_netdev_ops;
    net->flags |= IFF_ECHO;

    // 配置私有信息
	priv = netdev_priv(net);
    // can的时序配置的取值范围
    priv->can.bittiming_const = &bjhk_ipcan_bittiming_const;
    // 当切换CAN的模式时会被调用(如启动can busoff后的重启等)
    priv->can.do_set_mode = bjhk_ipcan_do_set_mode;
    // 通过设备树获取时钟 bjhk,can-clock-hz = <16000000>; 16000000即为获取的时钟信息
    ret = of_property_read_u32(spi->dev.of_node, "bjhk,can-clock-hz", &freq);
    if(ret) {
        dev_warn(&spi->dev, "IPCAN clock not specified, using default 16MHz\n");
        freq = 16000000; // fallback 默认值
    }
    // 告诉系统CAN的时钟配置
    priv->can.clock.freq = freq;
    // 只支持正常模式，不支持其他各种模式(不支持如 三采样、内部回环、监听模式等)
    priv->can.ctrlmode_supported = 0;
    // 配置用于错误计数器的函数
    priv->can.do_get_berr_counter = bjhk_ipcan_get_berr_counter;
    priv->net = net;
    
    // 将私有数据与spi驱动设备绑定
    spi_set_drvdata(spi, priv);

    /* -------------------配置SPI的相关内容-------------------*/
    /* 配置 SPI 模式、时钟等 */
    spi->mode = SPI_MODE_0; /* 具体模式取决于 IP 核 */
    if (!spi->max_speed_hz) /* 首先使用设备树的设置，否则默认5MHz */
        spi->max_speed_hz = IPCAN_SPI_FREQ;
    spi->bits_per_word = 16;
    if (spi_setup(spi)) {
        dev_err(&spi->dev, "bjhk_ipcan Failed to setup SPI\n");
        ret = -EINVAL;
        goto error_probe;
    }

    priv->spi = spi;
    // 初始化 SPI 互斥锁，防止多线程同时访问
    mutex_init(&priv->ipcan_lock);

    /* If requested, allocate DMA buffers */
	if (bjhk_ipcan_enable_dma) {
		spi->dev.coherent_dma_mask = ~0;

		/*
		 * Minimum coherent DMA allocation is PAGE_SIZE, so allocate
		 * that much and share it between Tx and Rx DMA buffers.
		 */
		priv->spi_tx_buf = dmam_alloc_coherent(&spi->dev,
						       PAGE_SIZE,
						       &priv->spi_tx_dma,
						       GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (priv->spi_tx_buf + (PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
							(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			bjhk_ipcan_enable_dma = 0;
		}
	}

    /* Allocate non-DMA buffers */
	if (!bjhk_ipcan_enable_dma) {
		priv->spi_tx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_probe;
		}
		priv->spi_rx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_rx_buf) {
			ret = -ENOMEM;
			goto error_probe;
		}
	}

    // 将net的绑定到spi，这样net设备就是spi的子设备，防止设备树关系错乱
    SET_NETDEV_DEV(net, &spi->dev);

    // 硬件探测
    ret = bjhk_ipcan_hw_probe(spi);
	if (ret) {
		if (ret == -ENODEV)
			dev_err(&spi->dev, "Cannot initialize IPCAN. Wrong wiring?\n");
		goto error_probe;
	}

    bjhk_ipcan_hw_sleep(spi);

    // 注册 CAN 设备
    ret = register_candev(net);
    if (ret) {
        dev_err(&spi->dev, "Failed to register CAN device\n");
        goto error_probe;
    }

    // 如果有CAN LED的话
    // devm_can_led_init(net);

    netdev_info(net, "BJHK IPCAN initialized. Clock: %dHz, DMA: %s\n",
            priv->can.clock.freq,
            bjhk_ipcan_enable_dma ? "enabled" : "disabled");
    return 0;

error_probe:
    free_candev(net);
    dev_err(&spi->dev, "ipcan Probe failed, err=%d\n", -ret);
    return ret;
}

static int bjhk_ipcan_remove(struct spi_device *spi)
{
    struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);

    unregister_candev(priv->net);
    free_candev(priv->net);

    return 0;
}

/*
CAN挂起驱动
*/
static int __maybe_unused bjhk_ipcan_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

    dev_info(dev, "IPCAN suspended\n");

	priv->force_quit = 1;
	disable_irq(spi->irq);
	/*
	 * Note: at this point neither IST nor workqueues are running.
	 * open/stop cannot be called anyway so locking is not needed
	 */
	if (netif_running(net)) {
		netif_device_detach(net);

		bjhk_ipcan_hw_sleep(spi);
		priv->after_suspend = AFTER_SUSPEND_UP;
	} else {
		priv->after_suspend = AFTER_SUSPEND_DOWN;
	}

	return 0;
}

/*
can恢复驱动
*/
static int __maybe_unused bjhk_ipcan_resume(struct device *dev)
{
	struct spi_device *spi;
	struct bjhk_ipcan_priv *priv;

    spi = to_spi_device(dev);
    priv = spi_get_drvdata(spi);

	if (priv->after_suspend & AFTER_SUSPEND_UP) {
        /* 
         mcp251x的驱动中这里只唤醒电源，并不会唤醒spi寄存器
         这个涉及到(defer处理)延迟唤醒 唤醒sleep寄存器(hw_wakeup)需要在restart_work_handler中做
         在suppend中可以触发sleep寄存器是因为suppend处于系统允许的sleep上下文
         而这里的的上下文可能不处于安全的唤醒环境,所以不直接写sleep寄存器进行唤醒操作
         resume() 不直接唤醒，是因为 SPI 操作不能在 atomic context 中做，
         而是交给 restart_work 来完成真正的唤醒和重初始化流程
         mcp251x 遵循这一标准设计，ipcan 也应该遵循这一标准设计
          */
		queue_work(priv->wq, &priv->restart_work);
	} else {
		priv->after_suspend = 0;
	}

	priv->force_quit = 0;
	enable_irq(spi->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(bjhk_ipcan_pm_ops, bjhk_ipcan_suspend,
                         bjhk_ipcan_resume);

static struct spi_driver bjhk_ipcan_can_driver = {
	.driver = {
		.name = DEVICE_NAME, /* 用于/sys/bus/spi/drivers/下注册DEVICE_NAME设备 */
		.of_match_table = bjhk_ipcan_of_match, /* 用于设备树匹配 */
        .pm = &bjhk_ipcan_pm_ops,  // 指定电源管理操作集
	},
	.id_table = bjhk_ipcan_id_table, /* 非设备树的id表匹配方式 */
    .probe = bjhk_ipcan_can_probe, /* spi设备加载回调函数 */
	.remove = bjhk_ipcan_remove, /* spi设备卸载回调函数 */
};
module_spi_driver(bjhk_ipcan_can_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("luoxl");
MODULE_DESCRIPTION("CPLD IPCAN Controller Driver");
