// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>

namespace solid_resident_test::heph_capture {
// Unchanged 10 N / dt=2^-20 actual-owner packet, diagnostic root e898b3e.
// solid-resident-heph-diag-tests-1.log SHA256 97aeaf72e950402bb24d47cbb244315cd2bbc77b01ffd5f2f6835fa51abec351
inline constexpr std::array<double,24> Position{
    0x0p+0, 0x0p+0, 0x0p+0,
    0x1.47ae147ae147bp-5, 0x0p+0, 0x0p+0,
    0x1.47ae147ae147bp-5, 0x1.eb851eb851eb8p-6, 0x0p+0,
    0x0p+0, 0x1.eb851eb851eb8p-6, 0x0p+0,
    0x0p+0, 0x0p+0, 0x1.47ae147ae147bp-6,
    0x1.47ae14aa8db44p-5, 0x0p+0, 0x1.47ae147ae147bp-6,
    0x1.47ae147ae147bp-5, 0x1.eb851eb851eb8p-6, 0x1.47ae147ae147bp-6,
    0x0p+0, 0x1.eb851eb851eb8p-6, 0x1.47ae147ae147bp-6
};
inline constexpr std::array<double,24> Velocity{
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x1.7d6364908018fp-12, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0
};
inline constexpr std::array<double,187> Native{
    0x1.82a29231dff42p-1, 0x1.4d5b10aa0c5a6p-1, 0x1.4d5b109dc3bc6p-1,
    -0x1.1c2808c2b0acap-4, 0x1.d7b149p-30, 0x1.aa3c0d240902ep-4,
    0x1.eeffffedfef02p+10, 0x1.cb544030b05a5p-30, 0x0p+0,
    -0x1.1c280938da10cp-8, 0x1.1c280938da10cp-8, -0x1.1c280938da10cp-8,
    -0x1.1c280938da10cp-8, 0x1.25fba8caecebep-37, -0x1.25fba8caecebep-37,
    0x1.25fba8caecebep-37, 0x1.25fba8caecebep-37, -0x1.4abb1e534a2cfp-36,
    0x1.4abb1e534a2cfp-36, -0x1.4abb1e534a2cfp-36, -0x1.4abb1e534a2cfp-36,
    0x1p-20, 0x1.5cdd89f66f83p-13, 0x1.1a7d2202b1795p-13,
    0x1.c3bafd475ac3cp-13, -0x1.c6f6bcc53bc72p-15, 0x1.20a9ae8cca7c7p-13,
    0x1.4a0d5cf053883p-13, -0x1.81e398cf7eefdp-14, -0x1.1a7d21d6c567fp-13,
    0x1.a7bbb2c02332cp-13, 0x1.b765b11009632p-14, -0x1.20a9ae638fa1cp-13,
    0x1.b0fe859454fdfp-13, 0x1.a60923f170843p-15, 0x1.b811d11397031p-14,
    -0x1.4a0d5ccfb3b6ap-13, -0x1.c90618993828fp-13, 0x1.2d275373760a1p-13,
    -0x1.c3bafd2f35f7fp-13, -0x1.74f8506f86607p-14, -0x1.b811d166fdf1fp-14,
    -0x1.b0fe85b23afb6p-13, 0x1.141f10ef396d3p-13, -0x1.2d27539ee97ebp-13,
    -0x1.a7bbb2db01d2ap-13, 0x1p+0, -0x1.8d4788bp-30,
    0x1.29f5a6e8p-29, 0x1.8d4788bp-30, 0x1p+0,
    0x1.ce65360e6be6p-60, -0x1.29f5a6e8p-29, 0x1.ce65360e6be6p-60,
    0x1p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x1.47ae147ae147bp-5, -0x1.fc8486p-35,
    0x1.7d6365p-34, 0x1.47ae1480d6d54p-5, 0x1.eb851ea86dc75p-6,
    0x1.7d63650377cc9p-34, 0x1.7d63648p-35, 0x1.eb851eb851eb8p-6,
    0x1.bbe6485fc3be1p-65, -0x1.7d6365p-35, 0x1.27eedaea827ecp-65,
    0x1.47ae147ae147bp-6, 0x1.47ae14a49826bp-5, -0x1.fc8486455bfb3p-35,
    0x1.47ae1492b77ep-6, 0x1.47ae147ae147bp-5, 0x1.eb851ea86dc75p-6,
    0x1.47ae1492b77ep-6, -0x1p-60, 0x1.eb851eb851eb8p-6,
    0x1.47ae147ae147bp-6, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x1.7d6364908018fp-12, -0x1.27eeda93fe3e1p-41,
    0x1.bbe64872f8306p-41, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x1.92a7371fb3782p-16, 0x1.47ae146ef62c7p-6,
    -0x1.8fffffe1492d8p+2, 0x1.900000019dd54p+2, 0x1.9000001b7b282p+2,
    -0x1.8fffffc76bdaap+2, -0x1.0aaaaaaf842a4p+3, -0x1.0aaaaaa5d12b2p+3,
    0x1.0aaaaaaf842a4p+3, 0x1.0aaaaaa5d12b2p+3, -0x1.8ffffff8b9c0bp+3,
    -0x1.90000007463f5p+3, -0x1.90000007463f5p+3, -0x1.8ffffff8b9c0bp+3,
    0x1.29f5a69af4535p-32, -0x1.29f5a6dd9efdfp-32, 0x1.29f5a6609efep-32,
    0x1.29f5a6609efep-32, -0x1.29f5a6705eee3p-32, 0x1.29f5a6f5b4438p-32,
    -0x1.29f5a678b4438p-32, -0x1.29f5a678b4438p-32, -0x1.29f5a5fe4dd06p-32,
    0x1.29f5a708f87b2p-32, -0x1.29f5a68bf87b1p-32, -0x1.29f5a68bf87b1p-32,
    0x1.29f5a70d05712p-32, -0x1.29f5a6ca5ac66p-32, 0x1.29f5a64d5ac67p-32,
    0x1.29f5a64d5ac67p-32, 0x1.47ae1486cc62dp-3, 0x1.eb851eb851eb8p-4,
    0x1.47ae147ae147bp-4, 0x1.29f5a664b125ap-29, -0x1.8d4788b73994dp-29,
    0x1.29f5a6896b2f9p-28, -0x1.ce653542a5ec6p-59, 0x1.344378fd18761p-58,
    -0x1.ce65357ba4b1p-58, 0x1.5acbe86660665p-58, -0x1.ce653616d4a28p-58,
    0x1.5acbe8911f79dp-57, 0x1.29f5a66153aa5p-9, -0x1.eec2a4p-68,
    0x1.aec6248p-63, -0x1.8d4788b2bcf05p-9, -0x1.07fda34p-63,
    0x1.29f5a6860db44p-8, 0x1.82a29231dff42p-1, 0x1.4d5b10aa0c5a6p-1,
    0x1.4d5b109dc3bc6p-1, -0x1.1c2808c2b0acap-4, 0x1.d7b149p-30,
    0x1.aa3c0d240902ep-4, 0x1.eeffffedfef02p+10, 0x1.3a937b4105c49p-30,
    0x0p+0, 0x1.82a29231dff42p-1, 0x1.4d5b10aa0c5a6p-1,
    0x1.4d5b109dc3bc6p-1, -0x1.1c2808c2b0acap-4, 0x1.d7b149p-30,
    0x1.aa3c0d240902ep-4, 0x1.ad5979f06239cp-1, 0x1.22a428d56dc56p-1,
    0x1p+0, 0x1.000000094fad2p+0, 0x1.a36e08deee8aap+8,
    0x1p+0, 0x0p+0, 0x1.29f5a6896b2f9p-28,
    0x1.344378fd18761p-57, 0x1.5acbe8911f79dp-56, -0x1.8d4788be7329ap-28,
    -0x1.ce6535c93ca9cp-56, 0x1.29f5a68ed65f3p-27, 0x1.d4a65ep-45,
    0x1.92a7371860deap-16, 0x1.eec9116c2748ap-46, 0x1.7c800dcdedbd2p-15,
    0x1.606cc1a79511p+24, 0x1.c75a8447c2cb4p-47, 0x1.6e36p+25,
    0x1.191238f3f547cp+2
};
inline constexpr std::array<double,21> Initial{
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x1.efp+10, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0,
    0x0p+0, 0x0p+0, 0x0p+0
};
inline constexpr std::array<double,6> InvariantStress{
    0.75514665925323704725768804060718925029626642671425, 0.65108548129275887489722553300449674042450582075107, 0.65108548154342390308157144501979311773236037751395,
    -0.069374118849206303785125708265998499602556235875347, -3.0079802599879584408588173406037742001839691973100e-10, 0.10406117827380944575152124762824442724375273534140
};
} // namespace solid_resident_test::heph_capture
