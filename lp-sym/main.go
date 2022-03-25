/* Copyright 2021, Arkadiusz Zarychta, arkadiusz.zarychta@h-brs.de */
/* Copyright 2021, Gurobi Optimization, LLC */

package main

import (
	"encoding/json"
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"git.solver4all.com/azaryc2s/op"
	"github.com/shirou/gopsutil/cpu"
	"github.com/shirou/gopsutil/host"
	"github.com/shirou/gopsutil/mem"
	"io/ioutil"
	"log"
	"os"
	"time"
)

var (
	edgeDist [][]int
	sol      op.Solution
	pInst    op.Instance
)

func main() {
	var (
		err error
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

	pInst.Solution = &sol
	edgeDist = op.CalcEdgeDist(pInst.NodeCoordinates, pInst.EdgeWeightType)

	// Create environment
	env, err := gurobi.LoadEnv("op-lp-sym.log")
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
	defer env.Free()
	threads, _ := env.GetIntParam(gurobi.INT_PAR_THREADS)
	sol.Comment = fmt.Sprintf("Using %d threads", threads)

	startTime := time.Now()

	tour, score, length, optimstatus, lb, err := op.SolveOP(edgeDist, pInst.Prices, pInst.TMax)
	if err != nil {
		log.Printf("Something went wrong while computing OP: %s\n", err.Error())
		return
	}

	sol.Time = time.Since(startTime).String()
	log.Println("\n---OPTIMIZATION DONE---\n\t Generating and writing result now\n")
	defer writeSolution()

	if optimstatus == gurobi.OPTIMAL {
		sol.Optimal = true
	} else if optimstatus == gurobi.INF_OR_UNBD {
		fmt.Printf("Model for %s is infeasible or unbounded\n", os.Args[1])
	} else if optimstatus == gurobi.TIME_LIMIT {
		sol.Comment += "Time limit reached"
	} else {
		sol.Comment += "For some reason the optimization stopped before the time limit without an optimal solution"
	}

	sol.Obj = score
	sol.LBound = lb
	sol.Route = tour
	sol.RouteCost = length
	fmt.Printf("Found a subtour with %d nodes and length %d: %v \n", len(tour), length, tour)
}

func writeSolution() {
	jsonInst, err := json.MarshalIndent(pInst, "", "\t")
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
	jsonInst = []byte(op.SanitizeJsonArrayLineBreaks(string(jsonInst)))
	//fileName := strings.ReplaceAll(os.Args[1], ".json", "_sol.json")
	fileName := os.Args[1]
	err = ioutil.WriteFile(fileName, jsonInst, 0644)
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
}
