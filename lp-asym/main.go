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
	"strings"
	"time"
)

var (
	N        int
	xijNum   int
	edgeDist [][]int
	sol      op.Solution
)

/* Define structure to pass data to the callback function */

type SubData struct {
	N int32
}

func main() {
	var (
		err   error
		pInst op.Instance
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
	env, err := gurobi.LoadEnv("op-lp-asym.log")
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
	defer env.Free()
	threads, _ := env.GetIntParam(gurobi.INT_PAR_THREADS)
	sol.Comment = fmt.Sprintf("Using %d threads", threads)

	N = pInst.Dimension
	xijNum = N * N //X_ij

	/* Create an empty model */

	model, err := env.NewModel("op", 0, nil, nil, nil, nil, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer model.Free()

	/* Add variables - one for every pair of nodes */
	/* Note: If edge from i to j is chosen, then x[i*n+j] = x[j*n+i] = 1. */
	/* The cost is split between the two variables. */

	for i := 0; i < N; i++ {
		for j := 0; j < N; j++ {
			name := fmt.Sprintf("x_%d_%d", i, j)
			err = model.AddVar(nil, nil, float64(pInst.Prices[i]), 0.0, 1.0, gurobi.BINARY, name)
			if err != nil {
				log.Println(err)
				return
			}
		}
	}

	// Change objective sense to maximization
	err = model.SetIntAttr(gurobi.INT_ATTR_MODELSENSE, gurobi.MAXIMIZE)
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}

	log.Println("Creating and setting depot constraints")
	{
		var (
			ind []int
			val []float64
		)
		for j := 0; j < N; j++ {
			ind = append(ind, 0*N+j)
			val = append(val, 1.0)
		}
		nameo := fmt.Sprintf("deg2o_depot")
		err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.EQUAL, 1, nameo)

		if err != nil {
			log.Println("Error adding depot_out_deg")
			log.Printf("At %s: %s\n", os.Args[1], err.Error())
			return
		}

		ind = nil
		val = nil
		for j := 0; j < N; j++ {
			ind = append(ind, j*N+0)
			val = append(val, 1.0)
		}
		namei := fmt.Sprintf("deg2i_depot")
		err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.EQUAL, 1, namei)

		if err != nil {
			log.Println("Error adding depot_in_deg")
			log.Printf("At %s: %s\n", os.Args[1], err.Error())
			return
		}
	}

	log.Println("Creating and setting constraints to forbid edge from node back to itself")
	{
		for i := 0; i < N; i++ {
			err = model.SetDblAttrElem(gurobi.DBL_ATTR_UB, int32(i*N+i), 0)
			if err != nil {
				log.Println(err)
				return
			}
		}
	}

	log.Println("Creating and setting constraints for nodes to only be visited once at max")
	{
		for j := 0; j < N; j++ {
			var (
				ind []int32
				val []float64
			)
			for i := 0; i < N; i++ {
				ind = append(ind, int32(i*N+j))
				val = append(val, 1.0)
			}
			err = model.AddConstr(ind, val, gurobi.LESS_EQUAL, 1.0, fmt.Sprintf("node_only1_%d", j))
			if err != nil {
				log.Printf("Error adding node_only1_%d\n", j)
				log.Printf("At %s: %s\n", os.Args[1], err.Error())
				return
			}
		}
	}
	log.Println("Creating and setting constraints for nodes to be left, if entered")
	{
		for j := 1; j < N; j++ {
			var (
				ind []int32
				val []float64
			)
			for i := 0; i < N; i++ {
				if i == j {
					continue
				}
				ind = append(ind, int32(i*N+j))
				val = append(val, 1.0)

				ind = append(ind, int32(j*N+i))
				val = append(val, -1.0)
			}
			err = model.AddConstr(ind, val, gurobi.EQUAL, 0.0, fmt.Sprintf("node_flow_%d", j))
			if err != nil {
				log.Println("Error adding node_flow_constraints")
				log.Printf("At %s: %s\n", os.Args[1], err.Error())
				return
			}
		}
	}

	log.Println("Creating and setting constraint for Tmax")
	{
		var (
			ind []int32
			val []float64
		)
		for i := 0; i < N; i++ {
			for j := 0; j < N; j++ {
				ind = append(ind, int32(i*N+j))
				val = append(val, float64(edgeDist[i][j]))
			}
		}
		err = model.AddConstr(ind, val, gurobi.LESS_EQUAL, float64(pInst.TMax), "travel_budget")
		if err != nil {
			log.Printf("Error adding constraint for travel budget")
			log.Printf("At %s: %s\n", os.Args[1], err.Error())
			return
		}
	}

	// Write model to a file with the same name as the input'
	lpName := strings.ReplaceAll(os.Args[1], ".json", ".lp-asym")
	err = model.Write(lpName)
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}

	/* Set callback function */

	err = model.SetCallbackFuncGo(subtourelimOP, SubData{N: int32(N)})
	if err != nil {
		log.Println(err)
		return
	}

	/* Must set LazyConstraints parameter when using lazy constraints */

	err = model.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		log.Println(err)
		return
	}

	startTime := time.Now()

	// Optimize model
	err = model.Optimize()
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
	sol.Time = time.Since(startTime).String()
	log.Println("\n---OPTIMIZATION DONE---\n\t Generating and writing result now\n")
	defer writeSolution()

	// Capture solution information
	optimstatus, err := model.GetIntAttr(gurobi.INT_ATTR_STATUS)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve optimization status: %s. ", err.Error())
		log.Printf("At %s: %s\n", os.Args[1], sol.Comment)
		return
	}

	if optimstatus == gurobi.OPTIMAL {
		sol.Optimal = true
	} else if optimstatus == gurobi.INF_OR_UNBD {
		fmt.Printf("Model for %s is infeasible or unbounded\n", os.Args[1])
	} else if optimstatus == gurobi.TIME_LIMIT {
		sol.Comment += "Time limit reached"
	} else {
		sol.Comment += "For some reason the optimization stopped before the time limit without an optimal solution"
	}

	objval, err := model.GetDblAttr(gurobi.DBL_ATTR_OBJVAL)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve the obj-value: %s. ", err.Error())
		log.Printf("At %s: %s\n", os.Args[1], sol.Comment)
		return
	}
	sol.Obj = int(objval)

	lb := 0.0
	lb, err = model.GetDblAttr(gurobi.DBL_ATTR_OBJBOUND)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve the lower-bound-value: %s. ", err.Error())
		log.Println(err)
	}
	sol.LBound = int(lb)

	solM, err := model.GetDblAttrMatrix(gurobi.DBL_ATTR_X, 0, int32(N))
	if err != nil {
		log.Println(err)
	}
	//fmt.Printf("Found Solution: %v \n", solM)
	solNodes := 0
	opLength := 0
	for i := 0; i < N; i++ {
		for j := 0; j < N; j++ {
			if solM[i][j] > 0.5 { //this count edges set to 1. Having n such edges, means we have n nodes to visit
				solNodes++
				opLength += edgeDist[i][j]
			}
		}
	}
	tour := findsubtour(solM)
	sol.Route = tour
	sol.RouteCost = opLength
	fmt.Printf("Found a subtour with %d nodes and length %d: %v \n", len(tour), opLength, tour)
}

/* Given an integer-feasible solution 'sol', find the smallest
   sub-tour.  Result is returned in 'tour', and length is
   returned in 'tourlenP'. */

func findsubtour(sol [][]float64) (result []int) {
	n := len(sol)
	seen := make([]bool, n)
	tour := make([]int, n)

	start := 0
	bestlen := n + 1
	bestind := -1
	i := 0
	node := 0
	for start < n {
		for node = 0; node < n; node++ {
			if !seen[node] {
				break
			}
		}
		if node == n {
			break
		}
		isConnected := false
		for leng := 0; leng < n; leng++ {
			tour[start+leng] = node
			seen[node] = true
			for i = 0; i < n; i++ {
				if sol[node][i] > 0.5 && !seen[i] {
					node = i
					isConnected = true
					break
				}
			}
			if i == n {
				leng++
				if isConnected && leng < bestlen {
					bestlen = leng
					bestind = start
				}
				start += leng
				break
			}
		}
	}

	return tour[bestind : bestind+bestlen]

}

/* Subtour elimination callback.  Whenever a feasible solution is found,
   find the shortest subtour, and add a subtour elimination constraint
   if that tour doesn't visit every node. */

func subtourelimOP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	n := usrdata.(SubData).N

	if where == gurobi.CB_MIPSOL {
		subSol, err := gurobi.CbGetDblMatrix(cbdata, where, gurobi.CB_MIPSOL_SOL, int(n))
		if err != nil {
			log.Println(err)
		}
		//fmt.Printf("Found subsolution: %v \n", subSol)
		subSolNodes := 0
		for i := 0; i < int(n); i++ {
			for j := 0; j < int(n); j++ {
				if subSol[i][j] > 0.5 { //this count edges set to 1. Having n such edges, means we have n nodes to visit
					subSolNodes++
				}
			}
		}
		//fmt.Printf("We should be looking for subtours with less than %d nodes\n", subSolNodes)
		tour := findsubtour(subSol)

		if len(tour) < subSolNodes {
			//fmt.Printf("Found a subtour with %d nodes\n", len(tour))
			//fmt.Printf("%v\n", tour)
			var (
				ind []int32
				val []float64
			)

			/* Add a subtour elimination constraint */
			for i := 0; i < len(tour); i++ {
				for j := 0; j < len(tour); j++ {
					if i == j {
						continue
					}
					ind = append(ind, int32(tour[i]*int(n)+tour[j]))
				}
			}
			for i := 0; i < len(ind); i++ {
				val = append(val, 1.0)
			}

			err = gurobi.CbLazy(cbdata, len(ind), ind, val, gurobi.LESS_EQUAL, float64(len(tour)-1))
			if err != nil {
				log.Println(err)
			}
		} else {
			fmt.Printf("Found a valid tour with %d nodes\n", len(tour))
			fmt.Printf("%v\n", tour)
		}
	}

	return 0
}

func writeSolution() {
	jsonInst, err := json.MarshalIndent(sol, "", "\t")
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
	jsonInst = []byte(op.SanitizeJsonArrayLineBreaks(string(jsonInst)))
	fileName := strings.ReplaceAll(os.Args[1], ".json", "_sol.json")
	err = ioutil.WriteFile(fileName, jsonInst, 0644)
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return
	}
}
