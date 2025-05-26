# MSI

## pci_enable_msi

```c
int pci_enable_msi(struct pci_dev *dev)
{
    int rc = __pci_enable_msi_range(dev, 1, 1, NULL) {

    }
    if (rc < 0)
        return rc;
    return 0;
}
```

# thermal

[WOWO tech :one:](http://www.wowotech.net/pm_subsystem/519.html) ⊙ [:two:](http://www.wowotech.net/pm_subsystem/520.html) ⊙ [:three:](http://www.wowotech.net/pm_subsystem/521.html) ⊙ [:four:](http://www.wowotech.net/pm_subsystem/522.html) ⊙ [:five:](http://www.wowotech.net/pm_subsystem/523.html)