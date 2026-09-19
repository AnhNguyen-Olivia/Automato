# Guide: Removing Ubuntu and Installing a Different Version (Dual-Boot with Windows)

This guide covers safely removing your current Ubuntu installation and its partitions, then installing a different Ubuntu version — **without touching your Windows partition or files**.

⚠️ **Back up any files in your Ubuntu `/home` directory first** (project code, datasets, etc.) to an external drive, USB, or cloud storage. Once you delete the Ubuntu partitions, that data is gone.

---

## Step 1: Boot into Windows

Before removing Ubuntu, boot into Windows first (from the GRUB menu) so you can later fix the boot loader from there if needed.

- Confirm Windows still works fine.
- Back up anything you need from Ubuntu (if you can still access it via GRUB) or from a live USB (see Step 2).

---

## Step 2: Boot from a Ubuntu Live USB

You'll use a live USB (your existing installer USB, or a fresh one for the new version) to safely delete partitions from outside the installed OS.

1. Plug in your Ubuntu USB installer.
2. Restart → press your boot menu key (commonly **F12**, **F2**, **Esc**, or **Del**).
3. Select the USB → choose **"Try Ubuntu"** (not Install yet).

---

## Step 3: Open GParted and Delete the Ubuntu Partitions

1. Open **GParted** (Activities → search "GParted", or install via `sudo apt install gparted` if using a minimal live environment).
2. Identify your Ubuntu partitions. They'll typically be:
   - **Root `/`** — Ext4, the size you assigned earlier (e.g. ~50 GB)
   - **Swap** — small partition, listed as "linux-swap"
   - **(optional) `/home`** — Ext4, if you created one separately
3. **Do NOT touch:**
   - Your Windows **NTFS** partition(s)
   - The **EFI System Partition** (small, ~100–500 MB, `fat32`) — this is shared by both Windows and Ubuntu's boot loader. Deleting it can break Windows boot too.
4. Right-click each Ubuntu partition (root, swap, home) → **Delete**.
5. Click the green ✔ **Apply** to commit the changes.
6. The space previously used by Ubuntu will now show as **unallocated**.

---

## Step 4: Fix the Boot Loader (Important — Windows May Not Boot Directly Yet)

Deleting Ubuntu removes the OS but **GRUB (the boot menu) may still try to load**, resulting in a `grub rescue>` error on restart. Fix this from Windows:

### Option A: Restore Windows Boot Manager (recommended, cleanest)
1. Boot from a **Windows installation USB** (or Windows recovery USB).
2. Choose **"Repair your computer"** → **Troubleshoot** → **Advanced options** → **Command Prompt**.
3. Run:
   ```
   bootrec /fixmbr
   bootrec /fixboot
   bootrec /rebuildbcd
   ```
4. Restart — Windows should now boot directly without GRUB.

### Option B: If you plan to reinstall Ubuntu right away
You can skip Option A — the new Ubuntu installer will reinstall GRUB automatically during setup, which will detect Windows and create a new dual-boot menu. Only use Option A if you want a clean Windows-only boot in between.

---

## Step 5: Download the New Ubuntu Version

1. Go to the official releases page for the version you want (e.g. `releases.ubuntu.com`).
2. Download the correct `.iso` (match it to what your course/ROS version needs — e.g. Ubuntu 24.04 for ROS 2 Jazzy, Ubuntu 22.04 for ROS 2 Humble).
3. Flash it to a USB using **Rufus** (Windows) or **balenaEtcher**.

---

## Step 6: Create Free Space for the New Install (if needed)

If the unallocated space from Step 3 is still there, you can reuse it directly. If not, or if you want a different size:

1. Boot into Windows.
2. Open **Disk Management** (or use GParted from a live USB again).
3. Shrink your Windows partition if you need more space, following the same process as your original setup (see your earlier dual-boot guide).

---

## Step 7: Install the New Ubuntu Version

1. Boot from the new Ubuntu USB.
2. Choose **"Try or Install Ubuntu"** → **Install Ubuntu**.
3. On **Installation type**, choose **"Something else"** (manual partitioning).
4. Select the **unallocated space** → click **+** → create:

   | Partition | Size | Type | Mount point |
   |---|---|---|---|
   | Root `/` | your chosen size (e.g. 50000 MB) | Ext4 | `/` |
   | Swap | 8000–16000 MB | swap area | — |
   | (optional) Home | remainder | Ext4 | `/home` |

5. Select the **existing EFI partition** → set **"Use as: EFI System Partition"** → make sure **format is NOT checked**.
6. Set **"Device for boot loader installation"** to the main disk (e.g. `/dev/sda`), not a specific partition.
7. Click **Install Now** → verify on the confirmation screen that **Windows/NTFS is not marked for formatting**.
8. Continue through timezone, username/password setup, then **Restart**.

---

## Step 8: Confirm Dual-Boot Works

1. Remove the USB when prompted.
2. You should see the **GRUB menu** listing both **Ubuntu** and **Windows**.
3. Boot into each once to confirm both work correctly.

---

## Quick Checklist

- [ ] Backed up Ubuntu `/home` data
- [ ] Booted from live USB
- [ ] Deleted only Ubuntu partitions (root, swap, home) in GParted — **not** Windows or EFI
- [ ] Fixed boot loader if needed (or skipped since reinstalling immediately)
- [ ] Downloaded correct new Ubuntu ISO version
- [ ] Flashed new USB installer
- [ ] Installed using "Something else" → correct partitions → EFI reused, not formatted
- [ ] Verified Windows partition untouched on confirmation screen
- [ ] Confirmed both OSes boot via GRUB
