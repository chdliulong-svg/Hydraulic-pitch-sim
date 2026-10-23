"""
CODESYS OPC UA 读写验证测试脚本
针对 sew_hydraulic_pitch_contorl_20261017 项目
"""
import asyncio
from asyncua import Client, ua

async def main():
    url = "opc.tcp://127.0.0.1:4840"
    print(f"[1/4] 连接 CODESYS OPC UA 服务器: {url} ...")
    
    async with Client(url=url) as client:
        print("  -> 连接成功！\n")
        
        # 1. 检查服务器命名空间
        print("[2/4] 获取服务器命名空间列表:")
        ns_array = await client.get_namespace_array()
        for idx, ns in enumerate(ns_array):
            print(f"  - Namespace[{idx}]: {ns}")
        print()
        
        # 2. 读取 GVL.b 和 Common 变量测试
        print("[3/4] 测试 OPC UA 读取功能:")
        node_b = client.get_node("ns=4;s=|var|CODESYS Control Win V3.PAC.GVL.b")
        val_b = await node_b.read_value()
        print(f"  - 成功读取 GVL.b = {val_b} (NodeId: {node_b.nodeid})")
        
        node_wtg = client.get_node("ns=4;s=|var|CODESYS Control Win V3.PAC.Common.WTG")
        wtg_children = await node_wtg.get_children()
        print(f"  - 成功读取 Common.WTG 子节点数量 = {len(wtg_children)}")
        for child in wtg_children:
            dname = (await child.read_display_name()).Text
            print(f"      * WTG.{dname}")
        print()
        
        # 3. 测试写并读回功能
        print("[4/4] 测试 OPC UA 写入与读回功能 (针对 GVL.a):")
        node_a = client.get_node("ns=4;s=|var|CODESYS Control Win V3.PAC.GVL.a")
        orig_val = await node_a.read_value()
        print(f"  - 原始值 GVL.a = {orig_val}")
        
        test_val = 88
        print(f"  - 正在写入测试数值: {test_val} ...")
        await node_a.write_value(ua.DataValue(ua.Variant(test_val, ua.VariantType.Int16)))
        
        readback_val = await node_a.read_value()
        print(f"  - 写入后立即读回: GVL.a = {readback_val}")
        assert readback_val == test_val, f"读写校验失败: 期望 {test_val}, 实际 {readback_val}"
        print("  -> 写入与读回一致性校验: PASS 通过！")
        
        # 还原原始值
        await node_a.write_value(ua.DataValue(ua.Variant(orig_val, ua.VariantType.Int16)))
        restored_val = await node_a.read_value()
        print(f"  - 还原原始值: GVL.a = {restored_val}\n")
        
    print("=" * 60)
    print("OPC UA 协议链路、读操作、写操作测试全部 PASS！")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(main())
