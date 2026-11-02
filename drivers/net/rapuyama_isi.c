// SPDX-License-Identifier: GPL-2.0-only
#include <linux/capability.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/if_phonet.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <net/phonet/phonet.h>

#define RYISI_MEDIA		0x15
#define RYISI_MAX_FRAME		65541
#define RYISI_QUEUE_HIGH	64
#define RYISI_QUEUE_LOW		32

struct ryisi {
	struct net_device *netdev;
	struct miscdevice misc;
	struct sk_buff_head tx;
	struct mutex reader;
	wait_queue_head_t wait;
	atomic_t attached;
	spinlock_t stats_lock;
	struct rtnl_link_stats64 stats;
};

static struct ryisi *endpoint;

static int ryisi_frame(const u8 *data, size_t size, bool guest)
{
	unsigned int length;

	if (size < 10 || size > RYISI_MAX_FRAME || data[0] != RYISI_MEDIA)
		return -EINVAL;
	length = guest ? get_unaligned_le16(data + 4) :
			 get_unaligned_be16(data + 4);
	return length + 6 == size ? 0 : -EMSGSIZE;
}

static void ryisi_wake_tx(struct ryisi *isi)
{
	if (atomic_read(&isi->attached) && netif_running(isi->netdev) &&
	    skb_queue_len(&isi->tx) < RYISI_QUEUE_LOW)
		netif_wake_queue(isi->netdev);
}

static netdev_tx_t ryisi_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ryisi *isi = netdev_priv(dev);
	unsigned long flags;
	bool drop = false;

	if (skb->protocol != htons(ETH_P_PHONET) || !pskb_may_pull(skb, 10) ||
	    ryisi_frame(skb->data, skb->len, false))
		drop = true;
	spin_lock_irqsave(&isi->tx.lock, flags);
	if (!atomic_read(&isi->attached))
		drop = true;
	if (drop) {
		spin_unlock_irqrestore(&isi->tx.lock, flags);
		spin_lock_irqsave(&isi->stats_lock, flags);
		isi->stats.tx_dropped++;
		spin_unlock_irqrestore(&isi->stats_lock, flags);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (isi->tx.qlen >= RYISI_QUEUE_HIGH) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&isi->tx.lock, flags);
		return NETDEV_TX_BUSY;
	}
	__skb_queue_tail(&isi->tx, skb);
	if (isi->tx.qlen >= RYISI_QUEUE_HIGH)
		netif_stop_queue(dev);
	spin_unlock_irqrestore(&isi->tx.lock, flags);
	wake_up_interruptible(&isi->wait);
	return NETDEV_TX_OK;
}

static int ryisi_net_open(struct net_device *dev)
{
	struct ryisi *isi = netdev_priv(dev);

	if (atomic_read(&isi->attached))
		netif_start_queue(dev);
	else
		netif_stop_queue(dev);
	return 0;
}

static int ryisi_net_stop(struct net_device *dev)
{
	struct ryisi *isi = netdev_priv(dev);

	netif_stop_queue(dev);
	skb_queue_purge(&isi->tx);
	return 0;
}

static void ryisi_stats(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct ryisi *isi = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&isi->stats_lock, flags);
	*stats = isi->stats;
	spin_unlock_irqrestore(&isi->stats_lock, flags);
}

static const struct net_device_ops ryisi_net_ops = {
	.ndo_open = ryisi_net_open,
	.ndo_stop = ryisi_net_stop,
	.ndo_start_xmit = ryisi_xmit,
	.ndo_get_stats64 = ryisi_stats,
};

static void ryisi_setup(struct net_device *dev)
{
	static const u8 media = RYISI_MEDIA;

	dev->netdev_ops = &ryisi_net_ops;
	dev->header_ops = &phonet_header_ops;
	dev->type = ARPHRD_PHONET;
	dev->flags = IFF_POINTOPOINT | IFF_NOARP;
	dev->hard_header_len = 1;
	dev->addr_len = 1;
	dev_addr_set(dev, &media);
	dev->mtu = 4000;
	dev->min_mtu = PHONET_MIN_MTU;
	dev->max_mtu = 65535;
	dev->tx_queue_len = RYISI_QUEUE_HIGH;
}

static int ryisi_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct ryisi *isi = container_of(misc, struct ryisi, misc);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	if (atomic_cmpxchg(&isi->attached, 0, 1))
		return -EBUSY;
	file->private_data = isi;
	netif_carrier_on(isi->netdev);
	ryisi_wake_tx(isi);
	return stream_open(inode, file);
}

static int ryisi_release(struct inode *inode, struct file *file)
{
	struct ryisi *isi = file->private_data;

	netif_carrier_off(isi->netdev);
	netif_tx_disable(isi->netdev);
	skb_queue_purge(&isi->tx);
	atomic_set(&isi->attached, 0);
	return 0;
}

static ssize_t ryisi_read(struct file *file, char __user *buffer, size_t count,
			  loff_t *offset)
{
	struct ryisi *isi = file->private_data;
	struct sk_buff *skb;
	unsigned char *packet;
	unsigned long flags;
	ssize_t ret;

	if (!count)
		return 0;
	ret = mutex_lock_interruptible(&isi->reader);
	if (ret)
		return ret;
	for (;;) {
		skb = skb_dequeue(&isi->tx);
		if (skb)
			break;
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		ret = wait_event_interruptible(isi->wait,
					       !skb_queue_empty(&isi->tx));
		if (ret)
			goto unlock;
	}
	if (count < skb->len) {
		ret = -EMSGSIZE;
		goto requeue;
	}
	packet = kmalloc(skb->len, GFP_KERNEL);
	if (!packet) {
		ret = -ENOMEM;
		goto requeue;
	}
	ret = skb_copy_bits(skb, 0, packet, skb->len);
	if (!ret) {
		put_unaligned_le16(get_unaligned_be16(packet + 4), packet + 4);
		ret = copy_to_user(buffer, packet, skb->len) ? -EFAULT :
							       skb->len;
	}
	kfree(packet);
	if (ret < 0)
		goto requeue;
	spin_lock_irqsave(&isi->stats_lock, flags);
	isi->stats.tx_packets++;
	isi->stats.tx_bytes += skb->len;
	spin_unlock_irqrestore(&isi->stats_lock, flags);
	consume_skb(skb);
	ryisi_wake_tx(isi);
	goto unlock;
requeue:
	spin_lock_irqsave(&isi->tx.lock, flags);
	if (netif_running(isi->netdev)) {
		__skb_queue_head(&isi->tx, skb);
		skb = NULL;
	}
	spin_unlock_irqrestore(&isi->tx.lock, flags);
	if (skb)
		dev_kfree_skb(skb);
unlock:
	mutex_unlock(&isi->reader);
	return ret;
}

static ssize_t ryisi_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *offset)
{
	struct ryisi *isi = file->private_data;
	struct sk_buff *skb;
	unsigned long flags;
	int ret;

	if (count < 10 || count > RYISI_MAX_FRAME)
		return -EMSGSIZE;
	if (!netif_running(isi->netdev))
		return -ENETDOWN;
	skb = netdev_alloc_skb(isi->netdev, count);
	if (!skb)
		return -ENOMEM;
	if (copy_from_user(skb_put(skb, count), buffer, count)) {
		ret = -EFAULT;
		goto discard;
	}
	ret = ryisi_frame(skb->data, count, true);
	if (ret)
		goto discard;
	put_unaligned_be16(get_unaligned_le16(skb->data + 4), skb->data + 4);
	skb->protocol = htons(ETH_P_PHONET);
	skb_reset_mac_header(skb);
	skb_pull(skb, 1);
	skb->pkt_type = PACKET_HOST;
	skb->ip_summed = CHECKSUM_NONE;
	ret = netif_rx(skb);
	spin_lock_irqsave(&isi->stats_lock, flags);
	if (ret == NET_RX_DROP) {
		isi->stats.rx_dropped++;
	} else {
		isi->stats.rx_packets++;
		isi->stats.rx_bytes += count;
	}
	spin_unlock_irqrestore(&isi->stats_lock, flags);
	return count;
discard:
	dev_kfree_skb(skb);
	return ret;
}

static __poll_t ryisi_poll(struct file *file, poll_table *wait)
{
	struct ryisi *isi = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &isi->wait, wait);
	if (!skb_queue_empty(&isi->tx))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (netif_running(isi->netdev))
		mask |= EPOLLOUT | EPOLLWRNORM;
	return mask;
}

static const struct file_operations ryisi_fops = {
	.owner = THIS_MODULE,
	.open = ryisi_open,
	.release = ryisi_release,
	.read = ryisi_read,
	.write = ryisi_write,
	.poll = ryisi_poll,
};

static int __init ryisi_init(void)
{
	struct net_device *dev;
	struct ryisi *isi;
	int ret;

	dev = alloc_netdev(sizeof(*isi), "phonet%d", NET_NAME_ENUM,
			   ryisi_setup);
	if (!dev)
		return -ENOMEM;
	isi = netdev_priv(dev);
	isi->netdev = dev;
	skb_queue_head_init(&isi->tx);
	mutex_init(&isi->reader);
	init_waitqueue_head(&isi->wait);
	spin_lock_init(&isi->stats_lock);
	atomic_set(&isi->attached, 0);
	netif_carrier_off(dev);
	ret = register_netdev(dev);
	if (ret)
		goto free;
	isi->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "rapuyama-isi",
		.fops = &ryisi_fops,
		.mode = 0600,
	};
	ret = misc_register(&isi->misc);
	if (ret) {
		unregister_netdev(dev);
		goto free;
	}
	endpoint = isi;
	pr_info("rapuyama-isi: %s, backend /dev/rapuyama-isi\n", dev->name);
	return 0;
free:
	free_netdev(dev);
	return ret;
}

static void __exit ryisi_exit(void)
{
	misc_deregister(&endpoint->misc);
	unregister_netdev(endpoint->netdev);
	skb_queue_purge(&endpoint->tx);
	free_netdev(endpoint->netdev);
}

module_init(ryisi_init);
module_exit(ryisi_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RapuYama ISI Phonet endpoint for a userspace firmware runtime");
