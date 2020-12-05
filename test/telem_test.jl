using Test
using BenchmarkTools

using DynamicsAndControl: LogDataSink, LogDataFlowId, SimulationDataset

function test1()
    log_sink = LogDataSink()

    t = 0.0
    a = 1.0
    b = 2.0
    idx = 0
    while t <= 10.0
        println("logging at t = $t")
        log!(log_sink, (:truth, :test), t, (; a, b))

        # repeat logs at same time instance and ensure they are correctly handled
        if idx % 3 > 0
            a = 2a + 3b
            b = 2b - a
            t += 1.0
        end
        idx += 1
    end

    logged = log_sink.data[LogDataFlowId((:truth, :test))]

    @test logged[:time] ≈ [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    @test logged[:a] ≈ [1.0, 8.0, 4.0, -28.0, -44.0, 68.0, 244.0, -28.0, -1004.0, -892.0, 3124.0]
    @test logged[:b] ≈ [2.0, -4.0, -12.0, 4.0, 52.0, 36.0, -172.0, -316.0, 372.0, 1636.0, 148.0]

    data = SimulationDataset(log_sink)

    @infiltrate

end

function test2()
    log_sink = LogDataSink()

    @test_throws ErrorException log!(log_sink, :test, 0.0, (a=1.0, b=2.0, time=0.0))
end

test1()
test2()
