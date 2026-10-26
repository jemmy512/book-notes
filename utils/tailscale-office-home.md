# Office Remote Access and VS Code AI Setup

## Connection layout

```text
Home Mac → Tailscale → Office Windows PC → office private servers
```

## 1. Office Windows configuration

Both computers must belong to the same office-approved Tailscale network.

In Administrator PowerShell, identify the Tailscale and office Ethernet interface indexes:

```powershell
Get-NetIPInterface -AddressFamily IPv4 | Format-Table InterfaceIndex, InterfaceAlias, ConnectionState, Forwarding
```

If forwarding is not already enabled, replace the placeholders with the actual indexes:

```powershell
Set-NetIPInterface -InterfaceIndex <TAILSCALE_INDEX> -AddressFamily IPv4 -Forwarding Enabled

Set-NetIPInterface -InterfaceIndex <ETHERNET_INDEX> -AddressFamily IPv4 -Forwarding Enabled
```

Advertise the current AI subnet and the additional SSH host:

```powershell
tailscale set --advertise-routes=10.x.x.x/16,10.x.x.x/32 --unattended
```

In the Tailscale admin console:
- https://console.tailscale.com/admin/machines
- Approve both routes for the Windows PC.
- Permit only authorized users/devices and required service ports through grants/ACLs.
- Coordinate firewall and routing changes with IT.

### Example ACL policy (addresses redacted)

Merge these entries into the existing `acls` array in the Tailscale access-control policy; do not overwrite unrelated policy settings. Replace the placeholders with approved addresses before saving:

```json
{
	"acls": [
		{
			"action": "accept",
			"src": ["<HOME_MAC_TAILSCALE_IP>"],
			"dst": [
				"<OFFICE_WINDOWS_TAILSCALE_IP>:*",
				"<OFFICE_AI_SUBNET_CIDR>:*",
				"<OFFICE_SSH_HOST_IP>/32:*"
			]
		},
		{
			"action": "accept",
			"src": ["*"],
			"proto": "1",
			"dst": ["*:*"]
		}
	]
}
```

- `<HOME_MAC_TAILSCALE_IP>`: the Mac's current Tailscale IPv4 address.
- `<OFFICE_WINDOWS_TAILSCALE_IP>`: the Windows PC's current Tailscale IPv4 address.
- `<OFFICE_AI_SUBNET_CIDR>`: the approved AI network prefix, including its CIDR mask.
- `<OFFICE_SSH_HOST_IP>`: the additional office SSH server's IPv4 address, without a CIDR mask.

The first entry allows the Mac to access all ports on the Windows PC and the listed office destinations. Access to Windows itself (such as RDP on TCP 3389) is separate from access to its advertised subnets. For least privilege, replace `:*` with the required service ports.

The second entry allows IPv4 ICMP (`proto: "1"`) from any source to any destination matched by the policy. It is broader than a Mac-to-office-only rule; retain it only if approved by IT. It does not grant TCP/UDP access.

ACL permissions do not approve subnet routes, enable Windows RDP, or override Windows Firewall. Device re-registration can change Tailscale IP addresses, requiring updates to these IP-based rules.

## 2. Home Mac Tailscale client configuration

### GUI app option

Connect using the Tailscale Mac app. The app variants normally accept approved subnet routes automatically. Closing the settings window leaves the connection running.

### CLI-only option (current setup)

Install the command-line client and daemon:

```bash
brew install --formula tailscale
```

Start the daemon as a system service that also starts at boot, before login:

```bash
sudo brew services start tailscale
```

Connect and explicitly accept the subnet routes advertised by Windows:

```bash
sudo "$(brew --prefix)/bin/tailscale" up --accept-routes
```

Follow the authentication URL if prompted. Using the Homebrew binary explicitly avoids accidentally invoking an alias for the GUI app. This installation may register as a new device with a different Tailscale IP and require administrator approval.

Verify the boot service and connection:

```bash
sudo brew services list
"$(brew --prefix)/bin/tailscale" status
```

The service should show `started` with user `root`. Authentication and route preferences persist; `tailscale up` does not need to run at every boot. Enter administrator passwords only directly in Terminal.

To stop the boot service:

```bash
sudo brew services stop tailscale
```

### Route and connectivity verification

```bash
route -n get 10.x.x.x
```

The office destinations should use the Tailscale tunnel interface (`utun` number can change). Confirm private HTTPS access independently of DNS and HTTP proxies:

## 3. Surge configuration

Add these rules to the existing `[Rule]` section before broader proxy rules and `FINAL`:

```ini
IP-CIDR,10.x.x.x/16,DIRECT,no-resolve
```

## 4. AI DNS and connectivity checks

### Surge hostname mapping

Add this entry to the existing `[Host]` section of the Surge profile (create the section if absent):

```ini
[Host]
api.x.com = 10.x.x.x
```

Add an explicit domain rule before broader proxy rules and `FINAL` in the existing `[Rule]` section:

```ini
DOMAIN,api.x.com,DIRECT
```

Surge resolves the hostname to the private IP for connections it handles. `DIRECT` uses macOS routing, including the approved Tailscale subnet route; it does not bypass Tailscale. `use-local-host-item-for-proxy` is not required for a `DIRECT` connection.

### System-wide hosts mapping

For applications resolving the hostname outside Surge, use a temporary Mac hosts override. Both mappings can coexist; keep their IP addresses consistent. Neither mapping creates a Tailscale route.

Edit `/etc/hosts`:

```bash
sudo nano /etc/hosts
```

Add one working address:

```text
10.x.x.x api.x.com
```

Flush DNS:

```bash
sudo dscacheutil -flushcache
sudo killall -HUP mDNSResponder
```

This pins one server and does not provide automatic failover. Remove the temporary entry once proper DNS is configured.

## 5. vs code AI config

```json
// < xxx > customize parts
{
	"name": "<CUSTOM_PROVIDER>",
	"vendor": "customendpoint",
	"apiKey": "<${input:chat.lm.secret.REDACTED}>",
	"apiType": "responses",
	"models": [
		{
			"id": "<MODEL_ID_1>",
			"name": "<MODEL_NAME_1>",
			"url": "<https://api.example.com/v1>",
			"toolCalling": true,
			"vision": false,
			"maxInputTokens": 872000,
			"maxOutputTokens": 128000
		}
	]
}
//  1M context = 872000 + 128000
```

## Reference

[Tailscale subnet-router documentation](https://tailscale.com/docs/features/subnet-routers)
