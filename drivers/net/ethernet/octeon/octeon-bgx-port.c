/**********************************************************************
 * Author: Cavium, Inc.
 *
 * Contact: support@cavium.com
 * This file is part of the OCTEON SDK
 *
 * Copyright (c) 2014 Cavium, Inc.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful, but
 * AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 * NONINFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * or visit http://www.gnu.org/licenses/.
 *
 * This file may also be available under a different license from Cavium.
 * Contact Cavium, Inc. for more information
 **********************************************************************/
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/if_vlan.h>

#include <asm/octeon/cvmx-helper.h>
#include <asm/octeon/cvmx-helper-util.h>
#include <asm/octeon/cvmx-helper-cfg.h>
#include <asm/octeon/cvmx-bgxx-defs.h>
#include <asm/octeon/cvmx-helper-bgx.h>
#include <asm/octeon/cvmx-pko3.h>
#include <asm/octeon/cvmx-gserx-defs.h>

#include "octeon-bgx.h"
#define ENABLE_10G_1G_SWITCH 

struct bgx_port_priv {
	int numa_node;
	int bgx_interface;
	int index; /* Port index on BGX block*/
	int ipd_port;
	int xiface;
	const u8 *mac_addr;
	struct phy_device *phydev;
	struct device_node *phy_np;
	struct net_device *netdev;
	struct mutex lock;
	unsigned int last_duplex;
	unsigned int last_link;
	unsigned int last_speed;
	struct delayed_work dwork;
	
	bool work_queued;
	int intf_index;
};

static struct workqueue_struct *check_state_wq;
static DEFINE_MUTEX(check_state_wq_mutex);
extern int octeon_sfp_update(struct net_device *netdev, uint32_t event, uint32_t speed, uint32_t duplex);
static struct bgx_port_priv *bgx_port_netdev2priv(struct net_device *netdev)
{
	struct bgx_port_netdev_priv *nd_priv = netdev_priv(netdev);
	return nd_priv->bgx_priv;
}

void bgx_port_set_netdev(struct device *dev, struct net_device *netdev)
{
	struct bgx_port_priv *priv = dev_get_drvdata(dev);

	if (netdev) {
		struct bgx_port_netdev_priv *nd_priv = netdev_priv(netdev);
		nd_priv->bgx_priv = priv;
	}

	priv->netdev = netdev;
}
EXPORT_SYMBOL(bgx_port_set_netdev);

int bgx_port_get_carrier_speed_duplex(struct net_device *netdev, int *carrier, int *speed, int *duplex)
{
	cvmx_helper_link_info_t link_info;
	struct bgx_port_priv *p = bgx_port_netdev2priv(netdev);

	link_info = cvmx_helper_link_get(p->ipd_port);
	*carrier = link_info.s.link_up;
	*speed = link_info.s.link_up ? link_info.s.speed : 0;
	*duplex = link_info.s.full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
	return 0;
}
EXPORT_SYMBOL(bgx_port_get_carrier_speed_duplex);

int bgx_port_ethtool_get_settings(struct net_device	*netdev,
				  struct ethtool_cmd	*cmd)
{
	cvmx_helper_link_info_t link_info;
	struct bgx_port_priv *p = bgx_port_netdev2priv(netdev);

	if (p->phydev)
		return phy_ethtool_gset(p->phydev, cmd);

	link_info = cvmx_helper_link_get(p->ipd_port);
	cmd->speed = link_info.s.link_up ? link_info.s.speed : 0;
	cmd->duplex = link_info.s.full_duplex ? DUPLEX_FULL : DUPLEX_HALF;

	return 0;
}
EXPORT_SYMBOL(bgx_port_ethtool_get_settings);

int bgx_port_ethtool_set_settings(struct net_device	*netdev,
				  struct ethtool_cmd	*cmd)
{
	struct bgx_port_priv *p = bgx_port_netdev2priv(netdev);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (p->phydev)
		return phy_ethtool_sset(p->phydev, cmd);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(bgx_port_ethtool_set_settings);

int bgx_port_ethtool_nway_reset(struct net_device *netdev)
{
	struct bgx_port_priv *p = bgx_port_netdev2priv(netdev);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (p->phydev)
		return phy_start_aneg(p->phydev);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(bgx_port_ethtool_nway_reset);

const u8 *bgx_port_get_mac(struct net_device *netdev)
{
	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev);
	return priv->mac_addr;
}
EXPORT_SYMBOL(bgx_port_get_mac);

int bgx_port_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct bgx_port_priv *p = bgx_port_netdev2priv(netdev);

	if (p->phydev)
		return phy_mii_ioctl(p->phydev, ifr, cmd);
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(bgx_port_do_ioctl);

static void bgx_port_write_cam(int numa_node, int interface, int index, int cam, const u8 *mac)
{
	int i;
	union cvmx_bgxx_cmr_rx_adrx_cam adr_cam;
	u64 m = 0;
	if (mac)
		for (i = 0; i < 6; i++)
			m |= (((u64)mac[i]) << ((5 - i) * 8));
	adr_cam.u64 = m;
	adr_cam.s.en = mac ? 1 : 0;
	adr_cam.s.id = index;
	cvmx_write_csr_node(numa_node, CVMX_BGXX_CMR_RX_ADRX_CAM(index * 8 + cam, interface), adr_cam.u64);
}

/* Set MAC address for the net_device that is attached. */
void bgx_port_set_rx_filtering(struct net_device *netdev)
{
	union cvmx_bgxx_cmrx_rx_adr_ctl adr_ctl;
	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev);
	int available_cam_entries, current_cam_entry;
	struct netdev_hw_addr *ha;

	available_cam_entries = 8;
	adr_ctl.u64 = 0;
	adr_ctl.s.bcst_accept = 1; /* Accept all Broadcast*/

	if ((netdev->flags & IFF_PROMISC) || netdev->uc.count > 7) {
		adr_ctl.s.cam_accept = 0; /* Reject CAM match */
		available_cam_entries = 0;
	} else {
		/* One CAM entry for the primary address, leaves seven
		 * for the secondary addresses.
		 */
		adr_ctl.s.cam_accept = 1; /* Accept CAM match */
		available_cam_entries = 7 - netdev->uc.count;
	}

	if (netdev->flags & IFF_PROMISC) {
		adr_ctl.s.mcst_mode = 1; /* Accept all Multicast */
	} else {
		if (netdev->flags & IFF_MULTICAST) {
			if ((netdev->flags & IFF_ALLMULTI) || netdev_mc_count(netdev) > available_cam_entries)
				adr_ctl.s.mcst_mode = 1; /* Accept all Multicast */
			else
				adr_ctl.s.mcst_mode = 2; /* Accept all Multicast via CAM */
		}
	}
	current_cam_entry = 0;
	if (adr_ctl.s.cam_accept) {
		bgx_port_write_cam(priv->numa_node, priv->bgx_interface, priv->index,
				   current_cam_entry, netdev->dev_addr);
		current_cam_entry++;
		netdev_for_each_uc_addr(ha, netdev) {
			bgx_port_write_cam(priv->numa_node, priv->bgx_interface, priv->index,
					   current_cam_entry, ha->addr);
			current_cam_entry++;
		}
	}
	if (adr_ctl.s.mcst_mode == 2) {/* Accept all Multicast via CAM */
		netdev_for_each_mc_addr(ha, netdev) {
			bgx_port_write_cam(priv->numa_node, priv->bgx_interface, priv->index,
					   current_cam_entry, ha->addr);
			current_cam_entry++;
		}
	}
	while (current_cam_entry < 8) {
		bgx_port_write_cam(priv->numa_node, priv->bgx_interface, priv->index,
				   current_cam_entry, NULL);
		current_cam_entry++;
	}
	cvmx_write_csr_node(priv->numa_node,
			    CVMX_BGXX_CMRX_RX_ADR_CTL(priv->index, priv->bgx_interface),
			    adr_ctl.u64);
}
EXPORT_SYMBOL(bgx_port_set_rx_filtering);

static void bgx_port_adjust_link(struct net_device *netdev)
{
	struct bgx_port_priv *p = bgx_port_netdev2priv(netdev);
	int link_changed = 0;
	unsigned int link, speed, duplex;

	mutex_lock(&p->lock);

	if (!p->phydev->link && p->last_link)
		link_changed = -1;

	if (p->phydev->link
	    && (p->last_duplex != p->phydev->duplex
		|| p->last_link != p->phydev->link
		|| p->last_speed != p->phydev->speed)) {
		link_changed = 1;
	}

	link = p->last_link = p->phydev->link;
	speed = p->last_speed = p->phydev->speed;
	duplex = p->last_duplex = p->phydev->duplex;

	mutex_unlock(&p->lock);

	if (link_changed != 0) {
		cvmx_helper_link_info_t link_info;
		if (link_changed > 0) {
			pr_info("%s: Link is up - %d/%s\n", netdev->name,
				p->phydev->speed,
				DUPLEX_FULL == p->phydev->duplex ?
				"Full" : "Half");
		} else {
			pr_info("%s: Link is down\n", netdev->name);
		}
		link_info.u64 = 0;
		link_info.s.link_up = link ? 1 : 0;
		link_info.s.full_duplex = duplex ? 1 : 0;
		link_info.s.speed = speed;
		if (!link) {
			netif_carrier_off(netdev);
			mdelay(50); /* Let TX drain.  FIXME really check that it is drained. */
		}
		cvmx_helper_link_set(p->ipd_port, link_info);
		if (link)
			netif_carrier_on(netdev);
	}
}

static void bgx_port_check_state(struct work_struct *work)
{
	struct bgx_port_priv		*priv;
	cvmx_helper_link_info_t		link_info;

	priv = container_of(work, struct bgx_port_priv, dwork.work);

	link_info = cvmx_helper_link_get(priv->ipd_port);
	if (priv->last_link != link_info.s.link_up) {
		priv->last_link = link_info.s.link_up;
		if (link_info.s.link_up){
			octeon_sfp_update(priv->netdev, SFP_LINK_UP, link_info.s.speed, link_info.s.full_duplex);
			pr_info("%s: Link is up - %d/%s\n",
				    priv->netdev->name, link_info.s.speed,
				    link_info.s.full_duplex ? "Full" : "Half");
		}	
		else{
			octeon_sfp_update(priv->netdev, SFP_LINK_DOWN, 0, 0);
			pr_info("%s: Link is down\n", priv->netdev->name);
		}
	}
//	mutex_lock(&priv->lock);
	if (priv->work_queued)
		queue_delayed_work(check_state_wq, &priv->dwork, HZ);
//	mutex_unlock(&priv->lock);
}

#define FIX_1G_SW_MODE 
#ifdef FIX_1G_SW_MODE
extern int gindex;
#define MAX_NETDEV_NUM 9
extern struct net_device *netdev_list[MAX_NETDEV_NUM];

int setup_sgmii_port_phy_mode(int bgx, int port)
{
	cvmx_bgxx_gmp_pcs_miscx_ctl_t pcs_miscx_ctl;
	cvmx_bgxx_cmrx_config_t cmrx_config;
	cvmx_bgxx_gmp_pcs_mrx_control_t gmp_pcs_mrx_control; 

	gmp_pcs_mrx_control.u64 = cvmx_read_csr_node(0, CVMX_BGXX_GMP_PCS_MRX_CONTROL(port,bgx));
	gmp_pcs_mrx_control.s.rst_an = 1;
	gmp_pcs_mrx_control.s.an_en =  1;
	gmp_pcs_mrx_control.s.pwr_dn = 0;

	cvmx_write_csr_node(0, CVMX_BGXX_GMP_PCS_MRX_CONTROL(port,bgx),gmp_pcs_mrx_control.u64);

	cmrx_config.u64 = cvmx_read_csr_node(0,CVMX_BGXX_CMRX_CONFIG(port,bgx));
	cmrx_config.s.enable = 1;
    cmrx_config.s.data_pkt_tx_en = 0;
	cmrx_config.s.data_pkt_rx_en = 0;
	cvmx_write_csr_node(0,CVMX_BGXX_CMRX_CONFIG(port,bgx), cmrx_config.u64);
										
	pcs_miscx_ctl.u64 = cvmx_read_csr_node(0, CVMX_BGXX_GMP_PCS_MISCX_CTL(port,bgx));
	pcs_miscx_ctl.s.mac_phy = 1;
	cvmx_write_csr_node(0, CVMX_BGXX_GMP_PCS_MISCX_CTL(port, bgx),pcs_miscx_ctl.u64);

	return 0;
}
EXPORT_SYMBOL(setup_sgmii_port_phy_mode);
#endif

#define REMAP_LOGIC_PORT 
#ifdef REMAP_LOGIC_PORT
extern int eth_remap[9];
extern int eth_inv_map[9];
#endif

#define USE_CONTROL_ETH_PWR
#define DEBUG
#ifdef DEBUG
#define DPRINT(format, args...) printk(KERN_ERR format , ##args)
#else
#define DPRINT(format, args...)
#endif

#ifdef USE_CONTROL_ETH_PWR
extern void set_bgx_led(int port, int on_off);
extern void set_sfp_power(int port, int on_off);

// on = 1, off =0
int set_pcs_power(int port, int speed ,int on_off)
{
    int bgx = 0;
    cvmx_bgxx_spux_control1_t spux_control1;
    cvmx_bgxx_gmp_pcs_mrx_control_t gmp_control;

    if(port >= 4 && port < 8){
        bgx = 1;
        port -= 4;
    }else if (port ==8){
        bgx = 2;
        port = 2;
    }
    else if ( port >= 8)
        return -1;

    if(speed){
        spux_control1.u64 = cvmx_read_csr_node(0, CVMX_BGXX_SPUX_CONTROL1(port, bgx));
        spux_control1.s.lo_pwr = !on_off;
        cvmx_write_csr_node(0, CVMX_BGXX_SPUX_CONTROL1(port, bgx), spux_control1.u64);
    }
    else{
        gmp_control.u64 = cvmx_read_csr_node(0, CVMX_BGXX_GMP_PCS_MRX_CONTROL(port, bgx));
        gmp_control.s.pwr_dn = !on_off;
        cvmx_write_csr_node(0, CVMX_BGXX_GMP_PCS_MRX_CONTROL(port, bgx), gmp_control.u64);
    }

    return 0;
}

// on = 1, off =0
int set_serdes_eie_detection(int port, int on_off){

    cvmx_gserx_rx_eie_deten_t rx_eie_deten;
    int gserx_id =2,port_id = port;
    int ret = 0 ,tmp;

    if(port >= 4 && port < 8){
        gserx_id = 3;
        port_id = port - 4;
    }else if (port == 8){
        gserx_id = 6;
        port_id = 0;
    }
    else if ( port >= 8)
        return -1;

    rx_eie_deten.u64 = cvmx_read_csr_node(0, CVMX_GSERX_RX_EIE_DETEN(gserx_id));
    tmp = rx_eie_deten.s.eiede;
    tmp &=~(0x1<<port_id);
    tmp |= on_off<<port_id;
    rx_eie_deten.u64 = tmp;
    cvmx_write_csr_node(0, CVMX_GSERX_RX_EIE_DETEN(gserx_id), rx_eie_deten.u64);
    return 0;


}

int set_eth_pwr(int bgx, int index, int speed, int on_off)
{
	int port = bgx * 4 + index;

	if(port == 10)
		port = 8;
	
	set_serdes_eie_detection(port, on_off);
	set_pcs_power(port, speed ,on_off);
	
	return 0;
}	
#endif 

int bgx_port_enable(struct net_device *netdev)
{
	bool dont_use_phy;
	union cvmx_bgxx_cmrx_config cfg;
	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev);
	cvmx_helper_link_info_t link_info;

	cfg.u64 = cvmx_read_csr_node(priv->numa_node, CVMX_BGXX_CMRX_CONFIG(priv->index, priv->bgx_interface));
	if (cfg.s.lmac_type == 0 || cfg.s.lmac_type == 5) {
		/* 1G */
#ifdef USE_CONTROL_ETH_PWR
		set_eth_pwr(priv->bgx_interface, priv->index, 0, 1);
#endif
		union cvmx_bgxx_gmp_gmi_txx_append tx_append;
		union cvmx_bgxx_gmp_gmi_txx_min_pkt min_pkt;

		tx_append.u64 = cvmx_read_csr_node(priv->numa_node,
						   CVMX_BGXX_GMP_GMI_TXX_APPEND(priv->index, priv->bgx_interface));
		tx_append.s.fcs = 1;
		tx_append.s.pad = 1;

		cvmx_write_csr_node(priv->numa_node,
				    CVMX_BGXX_GMP_GMI_TXX_APPEND(priv->index, priv->bgx_interface),
				    tx_append.u64);

		min_pkt.u64 = 0;
		/* packets are padded (without FCS) to MIN_SIZE + 1 in SGMII */
		min_pkt.s.min_size = 60 - 1;
		cvmx_write_csr_node(priv->numa_node,
				    CVMX_BGXX_GMP_GMI_TXX_MIN_PKT(priv->index, priv->bgx_interface),
				    min_pkt.u64);
	} else {
		/* 10G or higher */
#ifdef USE_CONTROL_ETH_PWR
		set_eth_pwr(priv->bgx_interface, priv->index, 1, 1);
#endif		
		union cvmx_bgxx_smux_tx_append tx_append;
		union cvmx_bgxx_smux_tx_min_pkt min_pkt;

		tx_append.u64 = cvmx_read_csr_node(priv->numa_node,
						   CVMX_BGXX_SMUX_TX_APPEND(priv->index, priv->bgx_interface));
		tx_append.s.fcs_d = 1;
		tx_append.s.pad = 1;

		cvmx_write_csr_node(priv->numa_node,
				    CVMX_BGXX_SMUX_TX_APPEND(priv->index, priv->bgx_interface),
				    tx_append.u64);

		min_pkt.u64 = 0;
		/* packets are padded(with FCS) to MIN_SIZE  in non-SGMII */
		min_pkt.s.min_size = 60 + 4;
		cvmx_write_csr_node(priv->numa_node,
				    CVMX_BGXX_SMUX_TX_MIN_PKT(priv->index, priv->bgx_interface),
				    min_pkt.u64);

	}

	switch (cvmx_helper_bgx_get_mode(priv->xiface, priv->index)) {
	case CVMX_HELPER_INTERFACE_MODE_XLAUI:
	case CVMX_HELPER_INTERFACE_MODE_XFI:
	case CVMX_HELPER_INTERFACE_MODE_10G_KR:
	case CVMX_HELPER_INTERFACE_MODE_40G_KR4:
		dont_use_phy = true;
		break;
#ifdef FIX_1G_SW_MODE
	case CVMX_HELPER_INTERFACE_MODE_SGMII:

#ifdef REMAP_LOGIC_PORT
		if (!strncmp(netdev->name, "eth0", 4)){
#else
		if (!strncmp(netdev->name, "eth8", 4)){
#endif			
			dont_use_phy = false;
		}	
		else{
			cvmx_helper_set_1000x_mode(priv->xiface, priv->index, true);
			setup_sgmii_port_phy_mode(priv->bgx_interface, priv->index);
			cvmx_helper_set_mac_phy_mode(priv->xiface, priv->index, true);
			dont_use_phy = true;

			link_info.s.link_up = 1;
			link_info.s.full_duplex = 1;
			if (OCTEON_IS_MODEL(OCTEON_CN78XX)) {
				link_info.s.speed = cvmx_qlm_get_gbaud_mhz_node(priv->numa_node, cvmx_qlm_lmac(priv->xiface, priv->index)) * 8 / 10;
			} else {
				link_info.s.speed = cvmx_qlm_get_gbaud_mhz(cvmx_qlm_lmac(priv->xiface, priv->index)) * 8 / 10;
			}
			cvmx_helper_link_set(priv->ipd_port, link_info);
		}	
		break;
#endif

	default:
		dont_use_phy = false;
		break;
	}

	if (priv->phy_np == NULL || dont_use_phy) {
		
		cvmx_helper_link_autoconf(priv->ipd_port);
#if 0
		netif_carrier_on(netdev);
#else
		netif_carrier_off(netdev);
#endif
		mutex_lock(&check_state_wq_mutex);
		if (!check_state_wq) {
			check_state_wq =
				alloc_workqueue("check_state_wq", WQ_UNBOUND |
						WQ_MEM_RECLAIM, 1);
		}
		mutex_unlock(&check_state_wq_mutex);
		if (!check_state_wq)
			return -ENOMEM;

//		mutex_lock(&priv->lock);
		INIT_DELAYED_WORK(&priv->dwork, bgx_port_check_state);
		queue_delayed_work(check_state_wq, &priv->dwork, 0);
		priv->work_queued = true;
//		mutex_unlock(&priv->lock);

		pr_info("%s: Link is not ready\n", netdev->name);
		octeon_sfp_update(priv->netdev, SFP_ADM_UP, 0, 0);

		return 0;
	} else {
		priv->phydev = of_phy_connect(netdev, priv->phy_np,
					      bgx_port_adjust_link, 0,
					      PHY_INTERFACE_MODE_SGMII);
		if (!priv->phydev)
			return -ENODEV;
	}

	netif_carrier_off(netdev);

	if (priv->phydev)
		phy_start_aneg(priv->phydev);

	return 0;
}
EXPORT_SYMBOL(bgx_port_enable);

int bgx_port_disable(struct net_device *netdev)
{
	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev);
	cvmx_helper_link_info_t link_info;
	
	if (priv->phydev)
		phy_disconnect(priv->phydev);
	priv->phydev = NULL;

	netif_carrier_off(netdev);
	link_info.u64 = 0;
	priv->last_link = 0;
	cvmx_helper_link_set(priv->ipd_port, link_info);

//	mutex_lock(&priv->lock);
	if (priv->work_queued) {
		cancel_delayed_work_sync(&priv->dwork);
		priv->work_queued = false;
	}
//	mutex_unlock(&priv->lock);
#ifdef USE_CONTROL_ETH_PWR
	if(cvmx_helper_interface_get_mode(priv->xiface) == CVMX_HELPER_INTERFACE_MODE_XFI)
		set_eth_pwr(priv->bgx_interface, priv->index, 1, 0);
	else if (cvmx_helper_interface_get_mode(priv->xiface) == CVMX_HELPER_INTERFACE_MODE_SGMII) 
		set_eth_pwr(priv->bgx_interface, priv->index, 0, 0);
#endif		
	octeon_sfp_update(priv->netdev, SFP_ADM_DOWN, 0, 0);
	return 0;
}
EXPORT_SYMBOL(bgx_port_disable);

int bgx_port_change_mtu(struct net_device *netdev, int new_mtu)
{
	union cvmx_bgxx_cmrx_config cfg;
	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev);
	int max_frame;

	if (new_mtu < 60 || new_mtu > 65392) {
		netdev_warn(netdev, "Maximum MTU supported is 65392\n");
		
		return -EINVAL;
	}

	netdev->mtu = new_mtu;

	max_frame = round_up(new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN, 8);

	cfg.u64 = cvmx_read_csr_node(priv->numa_node, CVMX_BGXX_CMRX_CONFIG(priv->index, priv->bgx_interface));
	if (cfg.s.lmac_type == 0 || cfg.s.lmac_type == 5)
		cvmx_write_csr_node(priv->numa_node,		/* 1G */
				    CVMX_BGXX_GMP_GMI_RXX_JABBER(priv->index, priv->bgx_interface),
				    max_frame);
	else
		cvmx_write_csr_node(priv->numa_node,		/* 10G or higher */
				    CVMX_BGXX_SMUX_RX_JABBER(priv->index, priv->bgx_interface),
				    max_frame);

	return 0;
}
EXPORT_SYMBOL(bgx_port_change_mtu);

void bgx_port_mix_assert_reset(struct net_device *netdev, int mix, bool v)
{
	u64 global_config;
	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev);
	u64 mask = 1ull << (3 + (mix & 1));
	union cvmx_bgxx_cmrx_config cfg;

	if (OCTEON_IS_MODEL(OCTEON_CN78XX_PASS1_X) && v == true) {
		/* Need to disable the mix before resetting the bgx-mix
		 * interface as not doing so confuses the other already up
		 * lmacs.
		 */
		cfg.u64 = cvmx_read_csr_node(priv->numa_node,
					     CVMX_BGXX_CMRX_CONFIG(priv->index,
						   priv->bgx_interface));
		cfg.s.mix_en = 0;
		cvmx_write_csr_node(priv->numa_node,
				    CVMX_BGXX_CMRX_CONFIG(priv->index,
							  priv->bgx_interface),
				    cfg.u64);
	}

	global_config = cvmx_read_csr_node(priv->numa_node,
					   CVMX_BGXX_CMR_GLOBAL_CONFIG(priv->bgx_interface));

	if (v)
		global_config |= mask;
	else
		global_config &= ~mask;

	cvmx_write_csr_node(priv->numa_node, CVMX_BGXX_CMR_GLOBAL_CONFIG(priv->bgx_interface), global_config);

	if (OCTEON_IS_MODEL(OCTEON_CN78XX_PASS1_X) && v == false) {
		cfg.u64 = cvmx_read_csr_node(priv->numa_node,
					     CVMX_BGXX_CMRX_CONFIG(priv->index,
						   priv->bgx_interface));
		cfg.s.mix_en = 1;
		cvmx_write_csr_node(priv->numa_node,
				    CVMX_BGXX_CMRX_CONFIG(priv->index,
							  priv->bgx_interface),
				    cfg.u64);
	}
}
EXPORT_SYMBOL(bgx_port_mix_assert_reset);
#ifdef ENABLE_10G_1G_SWITCH 
static struct platform_device *bgx_port_pdev[8] = {NULL};
static int bgx_port_index = 0;
#endif
int bgx_port_probe(struct platform_device *pdev)

{
	u64 addr;
	const u8 *mac;
	const __be32 *reg;
	u32 index;
	int r;
	struct bgx_port_priv *priv;
	int numa_node;

	reg = of_get_property(pdev->dev.parent->of_node, "reg", NULL);
	addr = of_translate_address(pdev->dev.parent->of_node, reg);
	mac = of_get_mac_address(pdev->dev.of_node);

	numa_node = (addr >> 36) & 0x7;

	r = of_property_read_u32(pdev->dev.of_node, "reg", &index);
	if (r)
		return -ENODEV;
#ifdef ENABLE_10G_1G_SWITCH
	if(bgx_port_index <= 8)
		priv = kzalloc_node(sizeof(*priv), GFP_KERNEL, numa_node);
#else
	priv = kzalloc_node(sizeof(*priv), GFP_KERNEL, numa_node);
#endif
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);
	priv->numa_node = numa_node;
	priv->bgx_interface = (addr >> 24) & 0xf;
	priv->index = index;
	priv->xiface = cvmx_helper_node_interface_to_xiface(numa_node, priv->bgx_interface);
	priv->ipd_port = cvmx_helper_get_ipd_port(priv->xiface, index);
	if (mac)
		priv->mac_addr = mac;

	cvmx_helper_set_mac_phy_mode(priv->xiface, priv->index, 
				of_get_property(pdev->dev.of_node,
					"cavium,sgmii-mac-phy-mode", NULL));
	cvmx_helper_set_1000x_mode(priv->xiface, priv->index,
				of_get_property(pdev->dev.of_node,
					"cavium,sgmii-mac-1000x-mode", NULL));

	priv->phy_np = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);

	if (priv->phy_np == NULL)
		cvmx_helper_set_port_phy_present(priv->xiface, priv->index, false);
	else
		cvmx_helper_set_port_phy_present(priv->xiface, priv->index, true);

	r = dev_set_drvdata(&pdev->dev, priv);

	if (priv->phy_np)
		__cvmx_helper_bgx_port_init(priv->ipd_port, 1);
	else
		__cvmx_helper_bgx_port_init(priv->ipd_port, 0);

	if (r)
		goto err;

#ifdef USE_CONTROL_ETH_PWR
	if(cvmx_helper_interface_get_mode(priv->xiface) == CVMX_HELPER_INTERFACE_MODE_XFI)
		set_eth_pwr(priv->bgx_interface, priv->index, 1, 0);
	else if (cvmx_helper_interface_get_mode(priv->xiface) == CVMX_HELPER_INTERFACE_MODE_SGMII) 
		set_eth_pwr(priv->bgx_interface, priv->index, 0, 0);
#endif		

	
#ifdef ENABLE_10G_1G_SWITCH
	if(bgx_port_index < 8)
		bgx_port_pdev[bgx_port_index++] = pdev;	
#endif

	dev_info(&pdev->dev, "Probed\n");
	return 0;
err:
	kfree(priv);
	return r;
}

static int bgx_port_remove(struct platform_device *pdev)
{
	struct bgx_port_priv *priv = dev_get_drvdata(&pdev->dev);
	kfree(priv);
	return 0;
}

#ifdef ENABLE_10G_1G_SWITCH
extern int bgx_probe(struct platform_device *pdev);

int bgx_port_off(int port)
{
	bgx_port_disable(netdev_list[port]);
	
}
EXPORT_SYMBOL(bgx_port_off);
extern void reset_bgx_ops(int xiface, int mode);
int bgx_port_reinit(int port ,int mode)
{

	struct bgx_port_priv *priv = bgx_port_netdev2priv(netdev_list[port]);
	
	reset_bgx_ops(priv->xiface, mode);
	cvmx_helper_interface_get_mode(priv->xiface);
	
	if(bgx_port_probe(bgx_port_pdev[port]))
	{
		DPRINT("Error occurs on bgx_port_reinit\n");
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(bgx_port_reinit);
#endif

static void bgx_port_shutdown(struct platform_device *pdev)
{
	return;
}

static struct of_device_id bgx_port_match[] = {
	{
		.compatible = "cavium,octeon-7890-bgx-port",
	},
	{
		.compatible = "cavium,octeon-7360-xcv",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bgx_port_match);

static struct platform_driver bgx_port_driver = {
	.probe		= bgx_port_probe,
	.remove		= bgx_port_remove,
	.shutdown       = bgx_port_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= KBUILD_MODNAME,
		.of_match_table = bgx_port_match,
	},
};

static int __init bgx_port_driver_init(void)
{
	int r;

	bgx_nexus_load();
	r =  platform_driver_register(&bgx_port_driver);
	return r;
}
module_init(bgx_port_driver_init);

static void __exit bgx_port_driver_exit(void)
{
	platform_driver_unregister(&bgx_port_driver);
	if (check_state_wq)
		destroy_workqueue(check_state_wq);
}
module_exit(bgx_port_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cavium Networks <support@caviumnetworks.com>");
MODULE_DESCRIPTION("Cavium Networks BGX Ethernet MAC driver.");
