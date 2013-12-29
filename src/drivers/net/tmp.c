
static void ks8851_tx_timeout(struct net_device *ndev)
{
    struct ks8851_net *ks = netdev_priv(ndev);

    if (netif_msg_timer(ks))
        dev_err(&ndev->dev, " tx timeout\n");

    ndev->stats.tx_errors++;
    /* can't restart safely under softirq */
    schedule_work(&ks->restart_work);
}
