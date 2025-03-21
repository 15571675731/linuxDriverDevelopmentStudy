#include <linux/spi/spi.h>


// spi通信使用的时钟频率
#define IPCAN_SPI_FREQ                  5000000

// 定义spi0的GPIO_DATA_IRQ脚
#define IPCAN_IRQ_GPIO                  92

// 回显缓冲区(硬件不支持则软件需要设置大于等于1,硬件支持则可以设置为0)
#define TX_ECHO_SKB_MAX                 1

// can包的最大数据长度
#define CAN_FRAME_MAX_DATA_LEN	8
// 传输BUF的最大数据长度 4个16bit读寄存器操作 + 8个CAN数据?
#define SPI_TRANSFER_BUF_LEN	(8 + CAN_FRAME_MAX_DATA_LEN)

// 驱动设备的名称
#define DEVICE_NAME "bjhk_ipcan"


/* 定义IPUG132_CAN的寄存器 */
// 功能选择寄存器(r/w)
#define IPCAN_REG_FS                    0X00
// can使能寄存器(r/w)
#define IPCAN_REG_CE                    0X01
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
#define IPCAN_REG_COMS                  0x07
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



// 一个枚举类,为以后可能会出现的多路can做准备
enum bjhk_ipcan_model {
	BJHK_IPCAN_0	= 0xA000,
    BJHK_IPCAN_1	= 0xA001,
};


struct bjhk_ipcan_priv {
	// 用于存储Linux SocketCAN 框架的CAN设备信息
	struct can_priv	   can;
	// Linux网络设备的抽象,socket can作为网络驱动设备,需要net_device进行数据收发
	struct net_device *net;
	// 用于存储spi设备线管的信息
	struct spi_device *spi;
	// 硬件型号
	enum bjhk_ipcan_model model;

	// 互斥锁,用于保护spi方位,避免多个线程同时访问can控制器,导致数据冲突
	struct mutex mcp_lock; /* SPI device lock */

	// spi发送缓冲区
	u8 *spi_tx_buf;
	// spi接收缓冲区
	u8 *spi_rx_buf;
	// spi发送DMA地址
	dma_addr_t spi_tx_dma;
	// spi接收DMA地址
	dma_addr_t spi_rx_dma;
    // 是否使用dma的标志位
    u8 use_dma;

	// 发送队列
	struct sk_buff *tx_skb;
	// 发送队列的长度
	int tx_queue_len;

	// 驱动使用的工作队列
	struct workqueue_struct *wq;
	// 用于处理can发送任务
	struct work_struct tx_work;
	// 用于处理can复位或者重启任务,如总线关闭后的重启
	struct work_struct restart_work;

	// 状态控制 如果出现严重错误，强制退出CAN控制器
	int force_quit;
	// 用于记录是否开启错误重发功能
	int restart_tx;

    // GPIO相关
    // GPIO 作为中断
    struct gpio_desc *irq_gpio;
    // 中断号
    int irq;
    // 中断处理工作队列
    struct work_struct irq_work;
};

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



// 清理can发送队列
static void bjhk_ipcan_clean(struct net_device *net)
{
    struct bjhk_ipcan_priv *priv = netdev_priv(net);

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
static int ipcan_spi_trans(struct spi_device *spi, int len)
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

    if (priv->use_dma) {  // 如果支持 DMA
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

    priv->spi_tx_buf[0] = ((2 << 14) | ((addr & 0x0F) << 10)); // 读命令 + 地址
    priv->spi_tx_buf[1] = 0x0000; // Dummy 数据
    priv->spi_tx_buf[2] = 0x0000; // 用于接收寄存器高 16bit
    priv->spi_tx_buf[3] = 0x0000; // 用于接收寄存器低 16bit

    int ret = ipcan_spi_trans(spi, 4*sizeof(u16));
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

    priv->spi_tx_buf[0] = ((1 << 14) | ((addr & 0x0F) << 10)); // 写命令 + 地址
    priv->spi_tx_buf[1] = (value >> 16) & 0xFFFF; // 高 16bit
    priv->spi_tx_buf[2] = value & 0xFFFF; // 低 16bit

    return ipcan_spi_trans(spi, 3*sizeof(u16));
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
    ret = ipcan_read_register(priv, IPCAN_REG_ERRCNT, &errcnt);
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
    ipcan设置模式的函数
*/
static int bjhk_ipcan_do_set_mode(struct net_device *net, enum can_mode mode)
{
    u32 ce_val;
    int timeout = 100
    struct bjhk_ipcan_priv *priv = netdev_priv(net);

    switch (mode) {
    case CAN_MODE_START:
        bjhk_ipcan_clean(net);  // 清除错误状态（如果需要）
        priv->can.state = CAN_STATE_ERROR_ACTIVE; // 设为正常工作状态

        /* 1. 复位 CAN 控制器 */
        ret = ipcan_write_register(priv, IPCAN_REG_CE, (1 << 1) | (1 << 0)); // `CE=1`, `SOFTRST=1`
        if (ret) {
            dev_err(&priv->spi->dev, "IPCAN : Failed to reset CAN controller\n");
            return ret;
        }

        /* 2. 轮询 `SOFTRST` 位，等待其自动清零 */
        do {
            usleep_range(100, 200); // 100~200us 小延时，减少 CPU 负担
            ret = ipcan_read_register(priv, IPCAN_REG_CE, &ce_val);
            if (ret) {
                dev_err(&priv->spi->dev, "IPCAN : Failed to read CE register\n");
                return ret;
            }
            if (!(ce_val & (1 << 0))) // `SOFTRST` 位是否清零
                break;
        } while (timeout--);

        if (timeout <= 0) {
            dev_err(&priv->spi->dev, "IPCAN : SOFTRST did not clear, reset timeout\n");
            return -ETIMEDOUT;
        }

        /* 3. 重新启动 CAN(发送 SPI 指令让 IPCAN 进入正常模式) */
        ret = ipcan_write_register(priv, IPCAN_REG_CE, (1 << 1)); // `CE=1` 启用 CAN
        if (ret) {
            dev_err(&priv->spi->dev, "IPCAN : Failed to start CAN controller\n");
            return ret;
        }

        priv->restart_tx = 1;
        queue_work(priv->wq, &priv->restart_work); // 触发 CAN 重新启动
        break;

    default:
        return -EOPNOTSUPP;
    }

    return 0;
}


// 打开can设备时调用
static int bjhk_ipcan_open(struct net_device *net)
{
    // 获取私有数据域
    struct bjhk_ipcan_priv *priv = netdev_priv(net);
    // 获取spi设备块
	struct spi_device *spi = priv->spi;
    // 设置中断触发标识 一次性触发中断和下降沿触发中断
	unsigned long flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
    // 用于获取返回值
	int ret;

    // 打开CAN设备
    ret = open_candev(net);
	if (ret) {
		dev_err(&spi->dev, "IPCAN : unable to set initial baudrate!\n");
		return ret;
	}

    // 加锁，防止并发访问
    mutex_lock(&priv->mcp_lock);

    // 清空TX相关变量
    // 设置运行状态标志位位正常状态
    priv->force_quit = 0;
    // 清空发送缓冲区,防止残留数据影响CAN发送
	priv->tx_skb = NULL;
    // 清空发送缓冲区长度,使任务从0开始
	priv->tx_len = 0;

    // 申请中断
    // 直接使用设备树中spi->irq配置的中断
    ret = request_threaded_irq(spi->irq, NULL, bjhk_ipcan_interrupt,
				   flags | IRQF_ONESHOT, DEVICE_NAME, priv);
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
	INIT_WORK(&priv->tx_work, mcp251x_tx_work_handler);
    // 初始化 restart_work 工作任务
	INIT_WORK(&priv->restart_work, mcp251x_restart_work_handler);

    // 复位IPCAN
	ret = mcp251x_hw_reset(spi);
	if (ret)
		goto out_free_wq;
	// 设置波特率、接收缓冲区、过滤模式等
	ret = mcp251x_setup(net, spi);
	if (ret)
		goto out_free_wq;
    // 使能中断，进入mormal模式
	ret = mcp251x_set_normal_mode(spi);
	if (ret)
		goto out_free_wq;
out_free_wq:
	destroy_workqueue(priv->wq);
out_clean:
	free_irq(spi->irq, priv);
	mcp251x_hw_sleep(spi);
out_close:
	mcp251x_power_enable(priv->transceiver, 0);
	close_candev(net);
	mutex_unlock(&priv->mcp_lock);
	return ret;
}


static const struct net_device_ops bjhk_ipcan_netdev_ops = {
    /* 负责启动CAN设备 */
	.ndo_open = bjhk_ipcan_open,
    /* 负责停止CAN设备 */
	.ndo_stop = bjhk_ipcan_stop,
    /* 负责发送CAN数据 */
	.ndo_start_xmit = bjhk_ipcan_hard_start_xmit,
};

// 设备树匹配表
static const struct of_device_id bjhk_ipcan_of_match[] = {
	{
		.compatible	= "bjhk,ipcan0",
		.data		= (void *)BJHK_IPCAN_0,
	},
    { }
};
MODULE_DEVICE_TABLE(of, bjhk_ipcan_of_match);

// 用于id匹配表
static const struct spi_device_id bjhk_ipcan_id_table[] = {
	{
		.name		= "ipcan0",
		.driver_data	= (kernel_ulong_t)BJHK_IPCAN_0,
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
    enum bjhk_ipcan_model model;
    struct net_device *net;
    struct bjhk_ipcan_priv *priv;
    int ret;
    int freq;

    /* -------------------一些通用初始化-------------------*/
    // 打印初始化信息
    dev_info(&spi->dev, "bjhk_ipcan CAN driver probing...\n");

    /* 通过设备树匹配表获取 CAN 设备类型 */
    match = of_match_device(bjhk_ipcan_of_match, &spi->dev);
    if (match && match->data)
        model = (enum bjhk_ipcan_model)match->data;
    else {
        dev_err(&spi->dev, "Failed to match device\n");
        return -ENODEV;
    }

    dev_info(&spi->dev, "Detected bjhk_ipcan CAN model: %x\n", model);

    /* TODO: 申请并初始化 CAN 设备、分配 buffer、注册 net_device */
    /* -------------------申请资源-------------------*/
    // 申请can dev
	net = alloc_candev(sizeof(struct bjhk_ipcan_priv), TX_ECHO_SKB_MAX);
	if (!net) {
        return -ENOMEM;
    }
	
    // 配置私有信息
	priv = netdev_priv(net);
	priv->spi = spi;
    priv->model = model;
	priv->net = net;
    // 将私有数据与spi驱动设备绑定
    spi_set_drvdata(spi, priv);
    // 将net的绑定到spi，这样net设备就是spi的子设备，防止设备树关系错乱
    SET_NETDEV_DEV(net, &spi->dev);
    
    /* -------------------配置SPI的相关内容-------------------*/
    /* 配置 SPI 模式、时钟等 */
    spi->mode = SPI_MODE_0; /* 具体模式取决于 IP 核 */
    spi->max_speed_hz = IPCAN_SPI_FREQ; /* 5MHz，视硬件支持情况调整 */
    spi->bits_per_word = 16;
    if (spi_setup(spi)) {
        dev_err(&spi->dev, "bjhk_ipcan Failed to setup SPI\n");
        ret = -EINVAL;
        goto err_free_netdev;
    }

    // 申请 SPI 发送和接收缓冲区 (可选，如果需要 DMA 传输)
    priv->spi_tx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN*sizeof(u16), GFP_KERNEL);
    priv->spi_rx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN*sizeof(u16), GFP_KERNEL);
    if (!priv->spi_tx_buf || !priv->spi_rx_buf) {
        dev_err(&spi->dev, "Failed to allocate SPI buffers\n");
        ret = -ENOMEM;
        goto err_free_netdev;
    }

    // 初始化 SPI 互斥锁，防止多线程同时访问
    mutex_init(&priv->mcp_lock);

    /* -------------------DMA相关的初始化-------------------*/
    // 默认不使用DMA
    priv->use_dma = 0;
    // DMA初始化相关内容

    /* -------------------CAN设备初始化-------------------*/
    // CAN的相关配置
    // 通过读取设备树中的 'can-clock' 信息获取CPLD配置的CAN时钟信息
    ret = of_property_read_u32(spi->dev.of_node, "can-clock", &freq);
    if (ret) {
        dev_err(&spi->dev, "Failed to read CAN clock from device tree\n");
        return -EINVAL;
    }
    // can的时序配置的取值范围
    priv->can.bittiming_const = &bjhk_ipcan_bittiming_const;
    // 当切换CAN的模式时会被调用(如启动can busoff后的重启等)
    priv->can.do_set_mode = bjhk_ipcan_do_set_mode;
    // 配置用于错误计数器的函数
    priv->can.do_get_berr_counter = bjhk_ipcan_get_berr_counter;
    // 告诉系统CAN的时钟配置
    priv->can.clock.freq = freq;
    // 只支持正常模式，不支持其他各种模式(不支持如 三采样、内部回环、监听模式等)
    priv->can.ctrlmode_supported = 0;
    net->netdev_ops = &bjhk_ipcan_netdev_ops;
    net->flags |= IFF_ECHO;

    /*
        这一段暂时报废，看能不能像mcp251x一样直接在设备树里面配置中断(在open里面注册中断,在stop里面关闭中断)
    */
    // /* -------------------GPIO、中断相关初始化-------------------*/
    // // 1. 获取 GPIO 
    // priv->irq_gpio = gpio_to_desc(IPCAN_IRQ_GPIO);
    // if (!priv->irq_gpio) {
    //     dev_err(&spi->dev, "ipcan: Failed to get GPIO %d\n", IPCAN_IRQ_GPIO);
    //     ret = -ENODEV;
    //     goto err_free_netdev;
    // }
    // // 2. 配置 GPIO 为输入模式
    // ret = gpiod_direction_input(priv->irq_gpio);
    // if (ret) {
    //     dev_err(&spi->dev, "ipcan: Failed to set GPIO %d as input\n", BJHK_IPCAN_IRQ_GPIO);
    //     goto err_free_netdev;
    // }
    // // 3. 获取中断号，并注册中断处理函数
    // priv->irq = gpiod_to_irq(priv->irq_gpio);
    // if (priv->irq < 0) {
    //     dev_err(&spi->dev, "Failed to get IRQ for GPIO %d\n", BJHK_IPCAN_IRQ_GPIO);
    //     ret = priv->irq;
    //     goto err_free_netdev;
    // }
    // // 注册中断回调
    // ret = devm_request_threaded_irq(&spi->dev, priv->irq, NULL,
    //                                 bjhk_ipcan_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
    //                                 "bjhk_ipcan", priv);
    // if (ret) {
    //     dev_err(&spi->dev, "Failed to request IRQ %d\n", priv->irq);
    //     goto err_free_netdev;
    // }

    // 注册 CAN 设备
    ret = register_candev(net);
    if (ret) {
        dev_err(&spi->dev, "Failed to register CAN device\n");
        goto err_free_netdev;
    }

    dev_info(&spi->dev, "bjhk_ipcan CAN driver successfully probed\n");
    return 0;

err_free_netdev:
    free_candev(net);
err_out:
    return ret;
}


static int bjhk_ipcan_remove(struct spi_device *spi)
{
    struct bjhk_ipcan_priv *priv = spi_get_drvdata(spi);

    unregister_candev(priv->net);
    free_candev(priv->net);

    return 0;
}


static struct spi_driver bjhk_ipcan_can_driver = {
	.driver = {
		.name = DEVICE_NAME, /* 用于/sys/bus/spi/drivers/下注册DEVICE_NAME设备 */
		.of_match_table = bjhk_ipcan_of_match, /* 用于设备树匹配 */
	},
	.id_table = bjhk_ipcan_id_table, /* 非设备树的id表匹配方式 */
    .probe = bjhk_ipcan_can_probe, /* spi设备加载回调函数 */
	.remove = bjhk_ipcan_can_remove, /* spi设备卸载回调函数 */
};
module_spi_driver(bjhk_ipcan_can_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("luoxl");
MODULE_DESCRIPTION("CPLD IPCAN Controller Driver");
