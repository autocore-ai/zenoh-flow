# Eclipse Zenoh-Flow Architecture

## graph

### flow

本流的唯一ID，不可重复

### operator

operators: 操作员们，有输入和输出，在graph中为输入输出的node

sources: 数据源头，只有输入，在graph中表现为只有输出的node

sinks: 数据输出点，只有输出，在graph中表现为只有输入的node

**注意：**

connectors: 

connector是自动生成的node，用于不同runtime的可跨网络，含有输入输出，在graph中表现为有输入输出的node，数据的传输通过zenoh实现

### links

带有方向的连接线，数据流向通过本连接传递，每个节点可有多条带有方向的links。要求输入输出的ID一致。

### mapping

每个operator对应的运行时名称，未配置时候，默认为启动时候的runtime名称
id: operator/source/sink/connector的id
runtime: 运行时名称

## 流程图

![avatar](./images/flow.png)


## 关于DataFlowGraph

![avatar](./images/dataflowgraph.png)

## 关于DataFlowRecord

![avatar](./images/dataflowrecord.png)

## 关于 Runner

### runner类型

![avatar](./images/runner.png)

### runner运行过程（待补充）



## 上下文如何实例化

![avatar](./images/zfcontext.png)

在ZFOperatorRunner/ZFSourceRunner/ZFSinkRunner的run方法中对ZFContext实例化
```
let ctx = ZFContext::new(self.operator.get_state(), 0);
```

实例化的上下文被传递给source/operator/sink的get_input_rule/get_run/get_output_rule中，以便运行时使用

### 上下文可用来做什么（待补充）

## 数据类型（待补充）

## Token（待补充）

## 创建graph不仅仅可以是yaml，也可以为json，或者代码直接实现各个operators（待补充）