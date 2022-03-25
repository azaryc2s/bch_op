package main

import (
	"encoding/json"
	"fmt"
	"git.solver4all.com/azaryc2s/op"
	"git.solver4all.com/azaryc2s/op/tsp"
	"github.com/shirou/gopsutil/cpu"
	"github.com/shirou/gopsutil/host"
	"github.com/shirou/gopsutil/mem"
	"io/ioutil"
	"log"
	"os"
)

func main() {
	var (
		err   error
		pInst op.Instance
		sol op.Solution
	)
	hostStat, _ := host.Info()
	cpuStat, _ := cpu.Info()
	vmStat, _ := mem.VirtualMemory()
	sol = op.Solution{Comment: "", System: op.SysInfo{hostStat.Platform, cpuStat[0].ModelName, fmt.Sprintf("%d GB", (vmStat.Total / 1024 / 1024 / 1024))}}

	instStr, err := ioutil.ReadFile(os.Args[1])

	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}

	err = json.Unmarshal(instStr, &pInst)

	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
	edgeDist := op.CalcEdgeDist(pInst.NodeCoordinates, pInst.EdgeWeightType)
	tour, length, _ := tsp.SolveTSP(edgeDist)
	sol.PInstance = pInst
	log.Printf("The calculated tour with length %d: %v",length, tour)
}
