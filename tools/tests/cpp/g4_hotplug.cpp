// 使用两对伪终端模拟同一路径拔出/重新枚举，输入仅来自伪网关。
#include "gateway_transport.hpp"
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <poll.h>
#include <pty.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using Gateway = ve33::GatewayTransport;
static void require(bool v, const char* reason) {
    if (!v) throw std::runtime_error(reason);
}
static int64_t now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static int replace_port(const std::filesystem::path& link) {
    char name[256]{};
    int master, slave;
    if (openpty(&master, &slave, name, nullptr, nullptr)) throw std::runtime_error("openpty 失败");
    close(slave);
    std::filesystem::remove(link);
    std::filesystem::create_symlink(name, link);
    return master;
}
int main(int argc, char **argv) {
    if (argc != 2) return 2;
    const std::filesystem::path link(argv[1]);
    const int master = replace_port(link);
    Gateway gateway(link.string());
    std::this_thread::sleep_for(220ms);
    require(gateway.poll(now()) == Gateway::Event::none, "无回包被误判为物理掉线");
    std::array<float, 60> tx{};
    for (int axis=0; axis<12; ++axis) tx[5*axis+4]=.25f;
    require(g4_set_mit_batch(gateway.handle(), tx.data(), 12, now()+90'000'000)==1, "假网关未接受测试批次");
    std::this_thread::sleep_for(12ms);
    close(master);
    auto deadline=now()+500'000'000;
    while (gateway.poll(now())!=Gateway::Event::lost) {
        require(now()<deadline, "串口挂断未报告");
        std::this_thread::sleep_for(2ms);
    }
    require(!gateway.handle(), "故障句柄没有撤下");
    require(gateway.poll(now())==Gateway::Event::none, "拨号退避未生效");
    const int replacement=replace_port(link);
    deadline=now()+2'500'000'000;
    while (gateway.poll(now())!=Gateway::Event::restored) {
        require(now()<deadline, "未重新打开原指定路径");
        std::this_thread::sleep_for(5ms);
    }
    require(gateway.handle() && gateway.reconnects()==1, "重新连接计数错误");
    // 新连接前几轮发给 12 个轴的必须全是查询，旧命令不能穿过重连。
    std::array<unsigned char, 32> buf{};
    int count=0;
    for(int i=0;i<24;++i) {
        size_t n=0;
        while(n<buf.size()) {
            pollfd fd{replacement,POLLIN,0};
            require(poll(&fd,1,500)==1,"未收到新连接的查询");
            auto got=read(replacement,buf.data()+n,buf.size()-n);
            require(got>0,"新串口断开");
            n+=static_cast<size_t>(got);
        }
        require(buf[0]==0xa5 && buf[1]==0x5a && buf[7]==2, "新设备收到了旧控制帧");
        ++count;
    }
    gateway.close();
    close(replacement);
    std::cout << count << " 个恢复后的网关帧均为只读查询\n";
}
