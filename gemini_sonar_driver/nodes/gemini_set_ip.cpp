/**
 * @file gemini_set_ip.cpp
 * @brief Standalone utility to inspect / program the alternate IP address of a
 *        Tritech Gemini sonar head, for commissioning a new head or moving one
 *        to a different vehicle subnet.
 *
 * Uses the low-level GeminiComms API (not Svs5Sequencer) because Svs5 exposes
 * no IP-configuration entry point. Only one process may bind the Gemini UDP
 * ports (52900/52901), so this tool cannot run while gemini_sonar_node (or
 * Genesis, or any other Gemini software) is running -- stop that first.
 *
 * A Gemini head ALWAYS responds on its fixed address 192.168.2.200; the value
 * programmed here is the *alternate* address it additionally responds to
 * after a reboot.
 *
 * Usage:
 *   ros2 run gemini_sonar_driver gemini_set_ip
 *       Discover sonars on the network and print their current addressing.
 *
 *   ros2 run gemini_sonar_driver gemini_set_ip --set <ip> <netmask> [options]
 *       Program the alternate IP + netmask, reboot the head, and verify the
 *       new address is reported in the head's status broadcasts.
 *
 * Options:
 *   --id <sonarId>   Target sonar ID (required if more than one head is seen)
 *   --via-alt        Send commands to the head's current alternate IP instead
 *                    of the fixed 192.168.2.200 (use when the host only has a
 *                    route to the alternate subnet)
 *   --yes            Skip the interactive confirmation prompt
 *
 * NOTE: the host must have an IP on the same subnet as the address used to
 * command the head (the library transmits on the adapter that best matches
 * the sonar's IP). For a factory-fresh head:
 *   sudo ip addr add 192.168.2.1/24 dev <iface>
 */

#include "DataTypes.h"
#include "Gemini/GeminiStructuresPublic.h"
#include "Gemini/GeminiCommsPublic.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct SonarInfo
{
  unsigned short sonar_id = 0;
  unsigned short firmware_ver = 0;
  unsigned char product_id = 0;
  unsigned int fix_ip = 0;
  unsigned int alt_ip = 0;
  unsigned int surface_ip = 0;
  unsigned int subnet_mask = 0;
  int status_count = 0;
};

std::mutex g_mutex;
std::map<unsigned short, SonarInfo> g_sonars;

std::string ipToString(unsigned int ip)
{
  std::ostringstream os;
  os << ((ip >> 24) & 0xFF) << "." << ((ip >> 16) & 0xFF) << "."
     << ((ip >> 8) & 0xFF) << "." << (ip & 0xFF);
  return os.str();
}

bool parseIp(const std::string & text, unsigned char octets[4])
{
  std::istringstream is(text);
  for (int i = 0; i < 4; ++i) {
    int value = -1;
    if (!(is >> value) || value < 0 || value > 255) {
      return false;
    }
    octets[i] = static_cast<unsigned char>(value);
    char dot;
    if (i < 3 && (!(is >> dot) || dot != '.')) {
      return false;
    }
  }
  return is.eof() || is.rdbuf()->in_avail() == 0;
}

bool isContiguousMask(const unsigned char m[4])
{
  unsigned int mask = (m[0] << 24) | (m[1] << 16) | (m[2] << 8) | m[3];
  // A valid mask is 1-bits followed by 0-bits: inverting gives 2^n - 1
  unsigned int inv = ~mask;
  return (inv & (inv + 1)) == 0;
}

const char * productName(unsigned char product_id)
{
  switch (product_id) {
    case GEM_HEADTYPE_720I: return "720i (Mk1)";
    case GEM_HEADTYPE_720ID: return "720id (Mk1)";
    case GEM_HEADTYPE_NBI: return "NBI (Mk1)";
    case GEM_HEADTYPE_MK2_720IS: return "720is (Mk2)";
    case GEM_HEADTYPE_MK2_1200IK: return "1200ik (Mk2)";
    case GEM_HEADTYPE_MK2_720IK: return "720ik (Mk2)";
    case GEM_HEADTYPE_720IM: return "720im";
    case GEM_HEADTYPE_MICRON_GEMINI: return "Micron Gemini";
    case GEM_HEADTYPE_MK2_1200ID: return "1200id (Mk2)";
    default: return "unknown";
  }
}

bool isMk1(unsigned char product_id)
{
  return product_id == GEM_HEADTYPE_720I || product_id == GEM_HEADTYPE_720ID ||
         product_id == GEM_HEADTYPE_NBI;
}

// GeminiComms callback. Only status packets matter here. MK1 and MK2 status
// packets share the layout of every field up to m_flags; the subnet mask sits
// at a different offset, so pick the struct by product ID. Mk2 DA-board
// status packets leave the IP fields zeroed -- skip those.
void gemCallback(int eType, int /*len*/, char * dataBlock)
{
  if (eType != GEM_STATUS || dataBlock == nullptr) {
    return;
  }
  const auto * status = reinterpret_cast<const CGemStatusPacket *>(dataBlock);
  if (status->m_sonarFixIp == 0) {
    return;
  }

  SonarInfo info;
  info.sonar_id = status->m_sonarId;
  info.firmware_ver = status->m_firmwareVer;
  info.product_id = static_cast<unsigned char>(status->m_flags >> 8);
  info.fix_ip = status->m_sonarFixIp;
  info.alt_ip = status->m_sonarAltIp;
  info.surface_ip = status->m_surfaceIp;
  if (isMk1(info.product_id)) {
    info.subnet_mask = status->m_subnetMask;
  } else {
    info.subnet_mask =
      reinterpret_cast<const CGemMk2BFStatusPacket *>(dataBlock)->m_subnetMask;
  }

  std::lock_guard<std::mutex> lock(g_mutex);
  auto & entry = g_sonars[info.sonar_id];
  info.status_count = entry.status_count + 1;
  entry = info;
}

std::map<unsigned short, SonarInfo> snapshotSonars()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_sonars;
}

void clearSonars()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_sonars.clear();
}

void printSonars(const std::map<unsigned short, SonarInfo> & sonars)
{
  for (const auto & [id, s] : sonars) {
    std::cout << "  Sonar ID " << id << "  [" << productName(s.product_id)
              << ", fw " << s.firmware_ver << "]\n"
              << "    fixed IP    : " << ipToString(s.fix_ip) << " (permanent)\n"
              << "    alternate IP: "
              << (s.alt_ip ? ipToString(s.alt_ip) : std::string("not programmed"))
              << "\n"
              << "    subnet mask : " << ipToString(s.subnet_mask) << "\n"
              << "    surface IP  : " << ipToString(s.surface_ip) << "\n";
  }
}

void usage()
{
  std::cout <<
    "Usage:\n"
    "  gemini_set_ip                              discover and list sonars\n"
    "  gemini_set_ip --set <ip> <netmask>         program alternate IP + reboot\n"
    "Options:\n"
    "  --id <sonarId>  target sonar (required with multiple heads online)\n"
    "  --via-alt       command the head via its current alternate IP\n"
    "  --yes           skip confirmation prompt\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  bool set_mode = false;
  bool via_alt = false;
  bool assume_yes = false;
  int target_id = -1;
  unsigned char new_ip[4] = {0, 0, 0, 0};
  unsigned char new_mask[4] = {0, 0, 0, 0};

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--set" && i + 2 < argc) {
      if (!parseIp(argv[i + 1], new_ip) || !parseIp(argv[i + 2], new_mask)) {
        std::cerr << "error: could not parse '" << argv[i + 1] << " "
                  << argv[i + 2] << "' as <ip> <netmask>\n";
        return 1;
      }
      set_mode = true;
      i += 2;
    } else if (arg == "--id" && i + 1 < argc) {
      target_id = std::atoi(argv[++i]);
    } else if (arg == "--via-alt") {
      via_alt = true;
    } else if (arg == "--yes") {
      assume_yes = true;
    } else if (arg == "--help" || arg == "-h") {
      usage();
      return 0;
    } else {
      std::cerr << "error: unrecognized argument '" << arg << "'\n";
      usage();
      return 1;
    }
  }

  if (set_mode) {
    const unsigned int ip_u32 =
      (new_ip[0] << 24) | (new_ip[1] << 16) | (new_ip[2] << 8) | new_ip[3];
    if (ip_u32 == 0) {
      std::cerr << "error: 0.0.0.0 is not a valid alternate IP\n";
      return 1;
    }
    if (ipToString(ip_u32) == "192.168.2.200") {
      std::cerr << "error: 192.168.2.200 is the permanent fixed address of "
                   "every Gemini head; pick a different alternate IP\n";
      return 1;
    }
    if (!isContiguousMask(new_mask)) {
      const unsigned int mask_u32 = (new_mask[0] << 24) | (new_mask[1] << 16) |
        (new_mask[2] << 8) | new_mask[3];
      std::cerr << "error: netmask " << ipToString(mask_u32)
                << " is not contiguous\n";
      return 1;
    }
  }

  GEM_SetHandlerFunction(&gemCallback);
  if (GEM_StartGeminiNetworkWithResult(0) == 0) {
    std::cerr <<
      "error: could not open the Gemini network ports.\n"
      "Another process using the Gemini library is probably running\n"
      "(gemini_sonar_node, its systemd service, or Genesis). Stop it and\n"
      "re-run this tool.\n";
    return 1;
  }
  GEM_SetGeminiSoftwareMode("EvoC");

  std::cout << "Listening for Gemini status broadcasts (UDP 52900)...\n";

  // Discovery: heads broadcast status once per second. Wait long enough to
  // hear every head at least twice; in set mode, bail out early once the
  // target is seen.
  const auto discover_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(set_mode ? 15 : 5);
  while (std::chrono::steady_clock::now() < discover_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (set_mode) {
      auto sonars = snapshotSonars();
      const SonarInfo * candidate = nullptr;
      if (target_id >= 0) {
        auto found = sonars.find(static_cast<unsigned short>(target_id));
        candidate = found != sonars.end() ? &found->second : nullptr;
      } else if (!sonars.empty()) {
        candidate = &sonars.begin()->second;
      }
      // Two statuses ensure the library has also learned the head's address
      if (candidate != nullptr && candidate->status_count >= 2) {
        break;
      }
    }
  }

  auto sonars = snapshotSonars();
  if (sonars.empty()) {
    std::cerr <<
      "error: no Gemini status messages received.\n"
      "Check the physical link, and that this host is on the head's L2\n"
      "segment. Status broadcasts arrive regardless of the host's subnet,\n"
      "so silence usually means a cabling/VLAN problem or the head is off.\n";
    GEM_StopGeminiNetwork();
    return 1;
  }

  std::cout << "\nDiscovered " << sonars.size() << " sonar(s):\n";
  printSonars(sonars);

  if (!set_mode) {
    GEM_StopGeminiNetwork();
    return 0;
  }

  // Resolve the target head
  if (target_id < 0) {
    if (sonars.size() > 1) {
      std::cerr << "\nerror: multiple sonars online; specify one with --id\n";
      GEM_StopGeminiNetwork();
      return 1;
    }
    target_id = sonars.begin()->first;
  }
  auto it = sonars.find(static_cast<unsigned short>(target_id));
  if (it == sonars.end()) {
    std::cerr << "\nerror: sonar ID " << target_id << " not seen on the network\n";
    GEM_StopGeminiNetwork();
    return 1;
  }
  const SonarInfo target = it->second;
  const unsigned short sonar_id = target.sonar_id;

  // The library's behavior depends on head type; set it before any command
  GEMX_SetHeadType(sonar_id, target.product_id);

  if (via_alt) {
    if (target.alt_ip == 0) {
      std::cerr << "error: --via-alt requested but the head has no alternate "
                   "IP programmed yet\n";
      GEM_StopGeminiNetwork();
      return 1;
    }
    GEMX_UseAltSonarIPAddress(
      sonar_id,
      (target.alt_ip >> 24) & 0xFF, (target.alt_ip >> 16) & 0xFF,
      (target.alt_ip >> 8) & 0xFF, target.alt_ip & 0xFF,
      (target.subnet_mask >> 24) & 0xFF, (target.subnet_mask >> 16) & 0xFF,
      (target.subnet_mask >> 8) & 0xFF, target.subnet_mask & 0xFF);
    GEMX_TxToAltIPAddress(sonar_id, 1);
  }

  const unsigned int new_ip_u32 =
    (new_ip[0] << 24) | (new_ip[1] << 16) | (new_ip[2] << 8) | new_ip[3];
  const unsigned int new_mask_u32 =
    (new_mask[0] << 24) | (new_mask[1] << 16) | (new_mask[2] << 8) | new_mask[3];

  std::cout << "\nAbout to program sonar ID " << sonar_id << " ("
            << productName(target.product_id) << "):\n"
            << "  alternate IP: "
            << (target.alt_ip ? ipToString(target.alt_ip) : std::string("none"))
            << " -> " << ipToString(new_ip_u32) << "\n"
            << "  subnet mask : " << ipToString(target.subnet_mask) << " -> "
            << ipToString(new_mask_u32) << "\n"
            << "  (commands sent to "
            << (via_alt ? ipToString(target.alt_ip) : ipToString(target.fix_ip))
            << ")\n\n"
            << "The head will be REBOOTED to apply the change. Do NOT power\n"
            << "cycle it during the process -- interrupting the flash write\n"
            << "can leave the head unusable.\n";

  if (!assume_yes) {
    std::cout << "Proceed? Type 'yes' to continue: " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "yes") {
      std::cout << "Aborted; nothing was written.\n";
      GEM_StopGeminiNetwork();
      return 0;
    }
  }

  std::cout << "\nProgramming alternate IP..." << std::endl;
  GEMX_SetAltSonarIPAddress(
    sonar_id,
    new_ip[0], new_ip[1], new_ip[2], new_ip[3],
    new_mask[0], new_mask[1], new_mask[2], new_mask[3]);

  // The interface spec requires >= 5 s for the flash write to complete before
  // a reboot; rebooting early corrupts the head's flash. Wait double that.
  std::cout << "Waiting 10 s for the head to commit the change to flash\n"
               "(spec minimum is 5 s -- do not interrupt)..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));

  std::cout << "Rebooting sonar..." << std::endl;
  clearSonars();
  GEMX_RebootSonar(sonar_id);

  std::cout << "Waiting for the head to come back (up to 90 s)..." << std::endl;
  const auto reboot_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(90);
  bool verified = false;
  SonarInfo after{};
  while (std::chrono::steady_clock::now() < reboot_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto now_sonars = snapshotSonars();
    auto post = now_sonars.find(sonar_id);
    if (post != now_sonars.end()) {
      after = post->second;
      if (after.alt_ip == new_ip_u32) {
        verified = true;
        break;
      }
    }
  }

  GEM_StopGeminiNetwork();

  if (verified) {
    std::cout << "\nSUCCESS: sonar ID " << sonar_id
              << " now reports alternate IP " << ipToString(after.alt_ip)
              << " / " << ipToString(after.subnet_mask) << "\n"
              << "The fixed address " << ipToString(after.fix_ip)
              << " remains active as well.\n";
    return 0;
  }

  if (after.status_count > 0) {
    std::cerr << "\nWARNING: head came back but reports alternate IP "
              << ipToString(after.alt_ip) << ", not "
              << ipToString(new_ip_u32) << ".\n"
                 "Re-run discovery to check its state before trying again.\n";
  } else {
    std::cerr << "\nWARNING: no status received after reboot. The head may\n"
                 "still be rebooting, or its broadcasts are not reaching this\n"
                 "host. Re-run this tool with no arguments to check on it.\n";
  }
  return 1;
}
