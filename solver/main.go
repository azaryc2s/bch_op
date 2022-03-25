/* Copyright 2021, Arkadiusz Zarychta, arkadiusz.zarychta@h-brs.de */
/* Copyright 2021, Gurobi Optimization, LLC */

package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"git.solver4all.com/azaryc2s/op"
	"git.solver4all.com/azaryc2s/op/tsp"
	"github.com/shirou/gopsutil/cpu"
	"github.com/shirou/gopsutil/host"
	"github.com/shirou/gopsutil/mem"
	"io/ioutil"
	"log"
	"math"
	"strings"
	"time"
)

const (
	Y_BOUNDS_CONT = "CONT"
	Y_BOUNDS_BIN  = "BIN"
	SEC           = "SEC"
	OP            = "OP"
	LBBD          = "LBBD"
	BCH           = "BCH"
	TSP           = "TSP"
	BEND_V0       = "BEND_V0"
	BEND_V1       = "BEND_V1"
	BEND_V2       = "BEND_V2"
)

var (
	N             int
	N0            int
	startX        int
	startY        int
	varCount      int
	edgeDist      [][]int
	sol           op.Solution
	pInst         op.Instance
	bendersCuts   int
	secCuts       int
	opCuts        int
	masterCbCount int
	cbData        MasterCallbackData
	cpuStat       []cpu.InfoStat
	hostStat      *host.InfoStat
	vmStat        *mem.VirtualMemoryStat

	cuts     op.ArrayStringFlags
	strat    *string
	subStrat *string
	inputF   *string
	outputF  *string
	yBounds  *string
)

/* Define structure to pass data to the callback function */

type MasterCallbackData struct {
	CurrentSolObj float64
	NewBestSol    bool
	NodeSequence  []int32
	TourLength    int
}

func main() {
	var err error

	flag.Var(&cuts, "cuts", "List of cuts to be used")
	strat = flag.String("strat", "BCH", "Strategy for solving. BCH (default) or LBBD")
	subStrat = flag.String("subStrat", "TSP", "Strategy for solving the subproblem. TSP (default) or OP")
	inputF = flag.String("input", "input.json", "Path to the input instance")
	yBounds = flag.String("yBounds", Y_BOUNDS_CONT, "Bounds of the Y-Variables. CONT (default) or BIN")
	outputF = flag.String("output", "", "Path to the output file. By default the input file will be overwritten adding the solution")

	flag.Parse()

	secCuts = 0
	bendersCuts = 0
	opCuts = 0
	hostStat, _ = host.Info()
	cpuStat, _ = cpu.Info()
	vmStat, _ = mem.VirtualMemory()
	sol = op.Solution{Comment: "", System: op.SysInfo{hostStat.Platform, cpuStat[0].ModelName, fmt.Sprintf("%d GB", (vmStat.Total / 1024 / 1024 / 1024))}}

	instStr, err := ioutil.ReadFile(*inputF)

	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}

	err = json.Unmarshal(instStr, &pInst)

	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}
	edgeDist = op.CalcEdgeDist(pInst.NodeCoordinates, pInst.EdgeWeightType)
	pInst.Solution = &sol

	// Create environment
	env, err := gurobi.LoadEnv(fmt.Sprintf("op-%s.log", *strat))
	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}
	defer env.Free()
	threads, _ := env.GetIntParam(gurobi.INT_PAR_THREADS)
	sol.Comment = fmt.Sprintf("Using %d threads", threads)

	N = pInst.Dimension
	N0 = N - 1

	/* Create an empty model */

	model, err := env.NewModel("op", 0, nil, nil, nil, nil, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer model.Free()

	/* Add variables X_i - one for every node*/
	log.Println("Adding variables X_i...")
	startX = 0
	varCount = 0
	for i := 0; i < N; i++ {
		name := fmt.Sprintf("X_%d", i)
		err = model.AddVar(nil, nil, float64(pInst.Prices[i]), 0.0, 1.0, gurobi.BINARY, name)
		if err != nil {
			log.Println(err)
			return
		}
		varCount++
	}
	startY = varCount

	/* Add variables Y_ij - one for every pair of nodes where j > i*/
	log.Println("Adding variables Y_i_j...")
	for i := 0; i < N; i++ {
		for j := i + 1; j < N; j++ {
			name := fmt.Sprintf("Y_%d_%d", i, j)
			var bounds int8
			if *yBounds == Y_BOUNDS_BIN {
				bounds = gurobi.BINARY
			} else if *yBounds == Y_BOUNDS_CONT {
				bounds = gurobi.CONTINUOUS
			}
			err = model.AddVar(nil, nil, 0.0, 0.0, 1.0, bounds, name)
			if err != nil {
				log.Println(err)
				return
			}
			varCount++
		}
	}

	// Change objective sense to maximization
	err = model.SetIntAttr(gurobi.INT_ATTR_MODELSENSE, gurobi.MAXIMIZE)
	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}

	log.Println("Creating and setting a constraint for the depot 0 to always be used")
	{
		ind := []int{startX}
		val := []float64{1.0}
		name := fmt.Sprintf("must_depot")
		err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.EQUAL, 1, name)
		if err != nil {
			log.Println("Error adding must_depot")
			log.Printf("At %s: %s\n", *inputF, err.Error())
			return
		}
	}

	log.Println("Creating and setting constraints for nodes to always be connected to 2 active edges")
	{
		for i := 0; i < N; i++ {
			var (
				ind []int32
				val []float64
			)
			for j := i + 1; j < N; j++ {
				ind = append(ind, int32(getYIndex(i, j)))
				val = append(val, 1.0)
			}
			for j := 0; j < i; j++ {
				ind = append(ind, int32(getYIndex(j, i)))
				val = append(val, 1.0)
			}
			ind = append(ind, int32(startX+i)) //X_i
			val = append(val, -2.0)
			err = model.AddConstr(ind, val, gurobi.EQUAL, 0.0, fmt.Sprintf("node_2_%d", i))
			if err != nil {
				log.Printf("Error adding node_2_%d\n", i)
				log.Printf("At %s: %s\n", *inputF, err.Error())
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
			for j := i + 1; j < N; j++ {
				ind = append(ind, int32(getYIndex(i, j)))
				val = append(val, float64(edgeDist[i][j]))
			}
		}
		err = model.AddConstr(ind, val, gurobi.LESS_EQUAL, float64(pInst.TMax), "travel_budget")
		if err != nil {
			log.Printf("Error adding constraint for travel budget")
			log.Printf("At %s: %s\n", *inputF, err.Error())
			return
		}
	}

	// Write model to a file with the same name as the input'
	lpName := strings.ReplaceAll(*inputF, ".json", ".lp")
	err = model.Write(lpName)
	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}

	/* Must set LazyConstraints parameter when using lazy constraints */

	err = model.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		log.Println(err)
		return
	}

	if *strat == BCH {
		solveByBCH(model)
	} else if *strat == LBBD {
		solveByLBBD(model)
	} else {
		log.Printf("Unsupported strategy: %s\n", *strat)
	}
	fmt.Printf("Found a OP-Tour with %d nodes, length %d and obj-Value of %d: %v \n", len(sol.Route), sol.RouteCost, sol.Obj, sol.Route)
}

func cutoffMasterSol(model *gurobi.Model, tourLength int, tour []int32, subtours [][]int32, objVal int) {
	// The master solution cannot be correct. Calculate values for the cut
	var err error
	for i := 0; i < len(cuts); i++ {
		cut := cuts[i]
		log.Printf("The Master solution with obj %d cannot be correct - Adding a benders cut %s to cut it off\n", objVal, cut)
		if cut == SEC {
			secInd, secVal, op, rhs := getSECs(subtours)
			for i := 0; i < len(secInd); i++ {
				err = model.AddConstr(secInd[i], secVal[i], op, rhs[i], fmt.Sprintf("%s_%d", cut, secCuts))
				if err != nil {
					log.Println(err)
				}
			}
			secCuts++
		}
		if cut == BEND_V0 {
			ind, val, op, rhs := getBendersCutV0(tour)
			// Add the benders cut
			err = model.AddConstr(ind, val, op, rhs, fmt.Sprintf("%s_%d", cut, bendersCuts))
			if err != nil {
				log.Printf("Error adding benders cut nr %d: %s\n", bendersCuts, err.Error())
			}
			bendersCuts++
		}
		if cut == BEND_V1 {
			ind, val, op, rhs := getBendersCutV1(tour, tourLength)
			// Add the benders cut
			err = model.AddConstr(ind, val, op, rhs, fmt.Sprintf("%s_%d", cut, bendersCuts))
			if err != nil {
				log.Printf("Error adding benders cut nr %d: %s\n", bendersCuts, err.Error())
			}
			bendersCuts++
		}
		if cut == BEND_V2 {
			ind, val, op, rhs := getBendersCutV2(tour, tourLength, pInst.TMax)
			for i := 0; i < len(ind); i++ {
				// Add the benders cut
				err = model.AddConstr(ind[i], val[i], op, rhs[i], fmt.Sprintf("%s_%d", cut, bendersCuts))
				if err != nil {
					log.Printf("Error adding benders cut nr %d\n", bendersCuts)
					return
				}
				bendersCuts++
			}
		}
	}
}

//calculate a heuristic tour with greedy strategy and set it as such for gurobi
func setHeuristicSol(model *gurobi.Model, cbData *MasterCallbackData, tour []int32, tourLength int, tourObj int, objVal int) {
	heurSol, newTourLength, heurObj := shortenTour(tour, edgeDist, pInst.Prices, tourLength, pInst.TMax, tourObj)

	if int(cbData.CurrentSolObj+0.5) < heurObj {
		cbData.CurrentSolObj = float64(heurObj)
		cbData.NodeSequence = heurSol
		cbData.TourLength = newTourLength

		if objVal == heurObj {
			//The current master-solution has the same objval as the calculated sequences from ATSP, so the value has been used already before we get the chance to set the solution!
			log.Printf("The current master-solution has the same objval %d as the calculated sequence from TSP", heurObj)
			cbData.NewBestSol = false
		} else if objVal > heurObj {
			//The heuristic solution is worse than the current objval, which means we will cut it off and start over
			log.Printf("Found new best solution with value %d, while the master solution was invalid", heurObj)
			cbData.NewBestSol = true
		} else {
			//The heuristic solution was better, than the master solution (this can happen??) HOW come??
			log.Printf("Found new best solution with value %d, which is even better than the current master solution!", heurObj)
			cbData.NewBestSol = true
		}
	}

	if cbData.NewBestSol {
		log.Printf("Currently setting new heuristic solution: %v with obj-value %.2f\n", cbData.NodeSequence, cbData.CurrentSolObj)
		if !checkSolutionValidity(cbData.NodeSequence, edgeDist, pInst.Prices, pInst.TMax, int(cbData.CurrentSolObj)) {
			log.Printf("Heuristic solution seems to be invalid!\n")
		}
		solution := make([]float64, varCount)

		//set the objective (X_i values)
		for i := 0; i < len(cbData.NodeSequence); i++ {
			solution[int32(startX)+cbData.NodeSequence[i]] = 1.0
		}

		//set the constraints (Y_ij values)
		for i := 0; i < len(cbData.NodeSequence); i++ {
			y := op.GetEdgeIndex(int(cbData.NodeSequence[i]), int(cbData.NodeSequence[(i+1)%len(cbData.NodeSequence)]), N, startY)
			solution[y] = 1.0
		}

		//set the solution
		err := model.SetDblAttrArray(gurobi.DBL_ATTR_START, 0, solution)

		//check the error and objv
		if err != nil {
			log.Printf("Couldn't set the heuristic solution: %s\n", err.Error())
		} else {
			cbData.NewBestSol = false
			log.Printf("New best starting solution with value : %d set!\n", int(cbData.CurrentSolObj+0.5))
		}
	}
}

func solveByLBBD(model *gurobi.Model) {
	var err error
	startTime := time.Now()
	solValid := false
	cbData = MasterCallbackData{NodeSequence: nil, NewBestSol: false, CurrentSolObj: 0, TourLength: 0}
	err = model.SetCallbackFuncGo(masterCallback, &cbData)
	if err != nil {
		log.Println(err)
		return
	}

	defer writeSolution()
	for !solValid {
		// Optimize model
		err = model.Optimize()
		if err != nil {
			log.Printf("At %s: %s\n", *inputF, err.Error())
			return
		}

		// Capture solution information
		optimstatus, err := model.GetIntAttr(gurobi.INT_ATTR_STATUS)
		if err != nil {
			sol.Comment += fmt.Sprintf("Couldn't retrieve optimization status: %s. ", err.Error())
			log.Printf("At %s: %s\n", *inputF, sol.Comment)
			return
		}

		if optimstatus == gurobi.OPTIMAL || optimstatus == gurobi.TIME_LIMIT {
			objvalF, err := model.GetDblAttr(gurobi.DBL_ATTR_OBJVAL)
			if err != nil {
				sol.Comment += fmt.Sprintf("Couldn't retrieve the obj-value: %s. ", err.Error())
				log.Printf("At %s: %s\n", *inputF, sol.Comment)
				return
			}

			objval := int(objvalF + 0.5)

			solA, err := model.GetDblAttrArray(gurobi.DBL_ATTR_X, 0, int32(varCount))
			if err != nil {
				sol.Comment += fmt.Sprintf("Couldn't retrieve the array with the decision variables: %s. ", err.Error())
				log.Printf("At %s: %s\n", *inputF, sol.Comment)
				return
			}
			if *subStrat == OP {
				xMat := extractNodeArray(solA)
				d, p, indx := transformToOP(xMat)
				opTour, heurObj, heurTourLength, _, _, err := op.SolveOP(d, p, pInst.TMax)

				//translate op tour to global indxs
				for k := 0; k < len(opTour); k++ {
					opTour[k] = indx[opTour[k]]
				}
				if heurObj < objval {
					activeNodes := extractActiveNodes(xMat)
					ind, val, op, rhs := getBendersCutOP(activeNodes, heurObj)
					// Add the benders cut
					err = model.AddConstr(ind, val, op, rhs, fmt.Sprintf("OP_%d", opCuts))
					if err != nil {
						log.Println(err)
					} else {
						opCuts++
					}
					setHeuristicSol(model, &cbData, gurobi.Int32Slice(opTour), heurTourLength, heurObj, objval)
				} else {
					//the OP-solution does not invalidate the master solution
					cbData.NodeSequence = gurobi.Int32Slice(opTour)
					cbData.CurrentSolObj = float64(objval)
					cbData.TourLength = heurTourLength
					solValid = true
					sol.Optimal = true
				}

			}
			if *subStrat == TSP {
				tour, tourLength, subtours := solveSubproblem(solA)

				if tour != nil && tourLength >= 0 && tourLength > pInst.TMax {
					cutoffMasterSol(model, tourLength, tour, subtours, objval)
					setHeuristicSol(model, &cbData, tour, tourLength, objval, objval)
				} else {
					//the TSP-solution does not invalidate the master solution
					cbData.NodeSequence = tour
					cbData.CurrentSolObj = float64(objval)
					cbData.TourLength = tourLength
					solValid = true
					sol.Optimal = true
				}
			}

			sol.Obj = int(cbData.CurrentSolObj + 0.5)
			sol.LBound = int(cbData.CurrentSolObj + 0.5)
			sol.UBound = objval

			if optimstatus == gurobi.TIME_LIMIT {
				sol.Comment += "Time limit reached"
				break
			}
		} else if optimstatus == gurobi.INF_OR_UNBD {
			fmt.Printf("Model for %s is infeasible or unbounded\n", *inputF)
			break
		} else {
			sol.Comment += "For some reason the optimization stopped before the time limit without an optimal solution"
			break
		}

	}
	sol.Time = time.Since(startTime).String()
	log.Println("\n---OPTIMIZATION DONE---\n\t Generating and writing result now\n")
	sol.Route = make([]int, len(cbData.NodeSequence))
	for i := 0; i < len(cbData.NodeSequence); i++ {
		sol.Route[i] = int(cbData.NodeSequence[i])
	}
	sol.RouteCost = cbData.TourLength
}

func solveByBCH(model *gurobi.Model) {
	var err error
	/* Set callback function */

	cbData = MasterCallbackData{NodeSequence: nil, NewBestSol: false, CurrentSolObj: 0, TourLength: 0}
	err = model.SetCallbackFuncGo(masterCallback, &cbData)
	if err != nil {
		log.Println(err)
		return
	}

	startTime := time.Now()
	// Optimize model
	err = model.Optimize()
	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}

	sol.Time = time.Since(startTime).String()
	log.Println("\n---OPTIMIZATION DONE---\n\t Generating and writing result now\n")
	defer writeSolution()

	// Capture solution information
	optimstatus, err := model.GetIntAttr(gurobi.INT_ATTR_STATUS)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve optimization status: %s. ", err.Error())
		log.Printf("At %s: %s\n", *inputF, sol.Comment)
		return
	}

	if optimstatus == gurobi.OPTIMAL {
		sol.Optimal = true
	} else if optimstatus == gurobi.INF_OR_UNBD {
		fmt.Printf("Model for %s is infeasible or unbounded\n", *inputF)
	} else if optimstatus == gurobi.TIME_LIMIT {
		sol.Comment += "Time limit reached"
	} else {
		sol.Comment += "For some reason the optimization stopped before the time limit without an optimal solution"
	}

	objval, err := model.GetDblAttr(gurobi.DBL_ATTR_OBJVAL)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve the obj-value: %s. ", err.Error())
		log.Printf("At %s: %s\n", *inputF, sol.Comment)
		return
	}
	sol.Obj = int(objval + 0.5)
	sol.LBound = int(objval + 0.5)

	ub := 0.0
	ub, err = model.GetDblAttr(gurobi.DBL_ATTR_OBJBOUND)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve the lower-bound-value: %s. ", err.Error())
		log.Println(err)
	}
	sol.UBound = int(ub)

	sol.Route = make([]int, len(cbData.NodeSequence))
	for i := 0; i < len(cbData.NodeSequence); i++ {
		sol.Route[i] = int(cbData.NodeSequence[i])
	}
	sol.RouteCost = cbData.TourLength
	checkSolutionValidity(gurobi.Int32Slice(sol.Route), edgeDist, pInst.Prices, pInst.TMax, sol.Obj)
}

func checkSolutionValidity(route []int32, d [][]int, p []int, tmax int, obj int) bool {
	routeLength := 0
	prices := 0
	for i := 0; i < len(route); i++ {
		j := (i + 1) % len(route)
		routeLength += d[route[i]][route[j]]
		prices += p[route[i]]
	}
	if routeLength <= tmax && prices == obj {
		log.Println("The computed solution is valid!")
		return true
	}
	if routeLength > tmax {
		log.Println("The computed solution is too long!")
		log.Printf("Is %d but can only be %d!\n", routeLength, tmax)
	}
	if prices != obj {
		log.Println("The computed solution has wrong obj value!")
		log.Printf("Solution has a obj value of %d but the solver says it has %d!\n", prices, obj)
	}
	return false
}

/* Given an integer-feasible solution 'sol', find the smallest
   sub-tour.  Result is returned in 'tour', and length is
   returned in 'tourlenP'. */

/*func findsubtour(sol [][]int) (result []int) {
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
				if sol[node][i] == 1 && !seen[i] {
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

}*/

/* Subtour elimination callback.  Whenever a feasible solution is found,
   find the shortest subtour, and add a subtour elimination constraint
   if that tour doesn't visit every node. */

/*func subtourelimOP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {

	if where == gurobi.CB_MIPSOL {
		subSol, err := gurobi.CbGetDblArray(cbdata, where, gurobi.CB_MIPSOL_SOL, varCount)
		if err != nil {
			log.Println(err)
		}
		yMat := extractEdgeMatrix(subSol)
		//fmt.Printf("Found subsolution: %v \n", subSol)
		subSolNodes := 0
		for i := startX; i < startY; i++ {
			if subSol[i] > 0.5 { //Counting the nodes to be visited
				subSolNodes++
			}
		}
		//fmt.Printf("We should be looking for subtours with less than %d nodes\n", subSolNodes)
		tour := findsubtour(yMat)

		if len(tour) < subSolNodes {
			//fmt.Printf("Found a subtour with %d nodes\n", len(tour))
			//fmt.Printf("%v\n", tour)
			var (
				ind []int32
				val []float64
			)

			// Add a subtour elimination constraint
			for i := 0; i < len(tour); i++ {
				for j := i + 1; j < len(tour); j++ {
					if tour[j] > tour[i] {
						ind = append(ind, int32(getYIndex(tour[i], tour[j])))
					} else {
						ind = append(ind, int32(getYIndex(tour[j], tour[i])))
					}

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
}*/

func transformToTSP(xMat []float64) ([][]int, []int) {
	indx := make([]int, 0)
	for i := 0; i < len(xMat); i++ {
		if xMat[i] > 0.5 {
			indx = append(indx, i)
		}
	}
	d := make([][]int, len(indx))
	for j := 0; j < len(d); j++ {
		a := indx[j]
		d[j] = make([]int, len(indx))
		for k := 0; k < len(d); k++ {
			if j == k {
				continue
			}
			b := indx[k]
			d[j][k] = edgeDist[a][b]
		}
	}
	return d, indx
}

func transformToOP(xMat []float64) (d [][]int, p []int, indx []int) {
	indx = make([]int, 0)
	for i := 0; i < len(xMat); i++ {
		if xMat[i] > 0.5 {
			indx = append(indx, i)
		}
	}
	d = make([][]int, len(indx))
	p = make([]int, len(indx))
	for j := 0; j < len(d); j++ {
		a := indx[j]
		d[j] = make([]int, len(indx))
		p[j] = pInst.Prices[a]
		for k := 0; k < len(d); k++ {
			if j == k {
				continue
			}
			b := indx[k]
			d[j][k] = edgeDist[a][b]
		}
	}
	return d, p, indx
}

func solveSubproblem(solArray []float64) ([]int32, int, [][]int32) {
	xMat := extractNodeArray(solArray)
	d, indx := transformToTSP(xMat)

	var (
		tour       []int32
		tourLength int
		subtours   [][]int32
	)
	if len(d) == 2 {
		//there are only 2 nodes assigned, we dont need to solve the tsp
		tourLength = d[0][1] * 2
		tour = []int32{0, 1}
	} else {
		tour, tourLength, subtours = tsp.SolveTSP(d)
		if tour == nil || tourLength < 0 {
			log.Println("For d the tour was nil. Why??:")
			op.Print2DArray(d)
			return nil, -1, nil
		}
	}

	//translate tsp tour to global indxs
	for k := 0; k < len(tour); k++ {
		tour[k] = int32(indx[tour[k]])
	}

	//translate tsp sub-tours to global indxs
	for j := 0; j < len(subtours); j++ {
		for k := 0; k < len(subtours[j]); k++ {
			subtours[j][k] = int32(indx[subtours[j][k]])
		}
	}

	return tour, tourLength, subtours
}

func findIntSubtour(edges [][]int) (result []int) {
	n := len(edges)
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
		subStart := node
		for leng := 0; leng < n; leng++ {
			tour[start+leng] = node
			seen[node] = true
			for i = 0; i < n; i++ {
				if edges[node][i] == 1 && !seen[i] {
					node = i
					isConnected = true
					break
				}
			}
			if i == n {
				leng++
				if isConnected && leng > 2 && edges[node][subStart] == 1 && leng < bestlen {
					bestlen = leng
					bestind = start
				}
				start += leng
				break
			}
		}
	}
	if bestind >= 0 && bestlen <= n {
		return tour[bestind : bestind+bestlen]
	}
	return nil

}

func getBendersCutV0(tour []int32) (ind []int32, val []float64, op int8, rhs float64) {
	//simply forbid the set of vertices (where X_i = 1)
	for j := 0; j < len(tour); j++ {
		ind = append(ind, int32(startX)+tour[j]) //this corresponds to the X_i Variables, since those start at 0 and end at n-1
		val = append(val, 1.0)
	}
	rhs = float64(len(tour) - 1)
	return ind, val, gurobi.LESS_EQUAL, rhs
}

/*CALCULATE AND ADD THE COMPLICATED BENDERS CUT
sum(Y_ij * d_ij) - sum_j(X_j * Theta_j)  >= TSP(V') - sum_j(Theta_j)
{i,j,k in V' ; i < j < k ; Y_ij = Y_jk = 1}
V' = subset of V with the nodes that are to be visited (for which the tsp is calculated)*/
func getBendersCutV1(tour []int32, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	for i := 0; i < len(tour); i++ {
		for j := i + 1; j < len(tour); j++ {
			ind = append(ind, int32(getYIndex(int(tour[i]), int(tour[j]))))
			val = append(val, float64(edgeDist[tour[i]][tour[j]]))
		}
	}

	edgeSum := 0
	for i := 1; i < len(tour); i++ {
		//u := i - 1
		//w := (i + 1) % len(tour)
		max := 0
		min := -1
		for j := 0; j < len(tour); j++ {
			next := edgeDist[tour[i]][tour[j]]
			if next > max {
				max = next
			}
			if min < 0 || next < min {
				min = next
			}
		}
		l := max*2 //2x the distance to the furthest node
		//l := edgeDist[u][i] + edgeDist[i][w]
		edgeSum += l
		ind = append(ind, int32(startX)+tour[i])
		val = append(val, float64(-l)) //we move it on the left side, so minus
	}
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - edgeSum)
}

/*CALCULATE AND ADD THE further+ improved BENDERS CUTs
it holds, that TSP(V') - L_sum <= TSP(V' \ {v_1,...,v_k})
L_sum = 2* ( (l_1+...+l_k+1) - max(l_1,...,l_k+1))
but it also already holds for L_sum = (l_1+...+l_k+1)
*/
func getBendersCutV2(tour []int32, tourLength int, tmax int) (ind [][]int32, val [][]float64, op int8, rhs []float64) {
	for i := 1; i < len(tour); i++ {
		count := 0
		st := i - 1

		currentMax := edgeDist[tour[st]][tour[i]]
		var edgesSet []int
		edgesSet = append(edgesSet, currentMax) //start with the edge y_j-1_j for the node x_j
		nodesLeft := make([]int32, len(tour))
		nodeVal := make([]float64, len(tour))
		//at the start, we add all nodes from the tour
		for t := 0; t < len(tour); t++ {
			nodesLeft[t] = tour[t]
			nodeVal[t] = 1.0
		}
		for j := i; j < len(tour); j++ {
			k := (j + 1) % len(tour)
			djk := edgeDist[tour[j]][tour[k]]
			edgesSet = append(edgesSet, djk) //add the edge y_j_k for the node x_j
			if djk > currentMax {
				currentMax = djk
			}
			Lsum1 := 0
			//Variant 1 - remove the longest edge and run over all remaining edges twice
			for s := 0; s < len(edgesSet); s++ {
				Lsum1 += edgesSet[s]
			}
			Lsum3 := Lsum1 //Variant 3 - simply sum the edges
			Lsum1 = 2 * (Lsum1 - currentMax)

			Lsum := int(math.Min(float64(Lsum1), float64(Lsum3)))

			//Variant 2 - sum the edges between the nodes, and add the shortest one to the rest + direct edge back
			Lsum2 := 0
			for s := 1; s < len(edgesSet)-1; s++ {
				Lsum2 += edgesSet[s]
			}
			if edgeDist[tour[st]][tour[i]] <= djk {
				Lsum2 += edgeDist[tour[st]][tour[i]]
				Lsum2 += edgeDist[tour[j]][tour[st]]
			} else {
				Lsum2 += djk
				Lsum2 += edgeDist[tour[i]][tour[k]]
			}

			Lsum = int(math.Min(float64(Lsum), float64(Lsum2)))

			if tourLength-Lsum <= tmax {
				//At this point it should hold: TSP(V) - Lsum <= TSP(V\{v_k,...,v_j})
				//and since we are already under tmax with our lower bound
				//it could be (maybe) possible to construct a viable TSP if we removed node x_j
				//so we stop here and forbid the node set V\{v_i,...,v_j-1} since it was still > tmax even for our lower bound

				nodesLeft = append(nodesLeft[:i], nodesLeft[j:]...)
				nodeVal = append(nodeVal[:i], nodeVal[j:]...)
				break
			} else {
				count++
			}
		}
		ind = append(ind, nodesLeft)
		val = append(val, nodeVal)
		rhs = append(rhs, float64(len(nodesLeft)-1))
	}
	return ind, val, gurobi.LESS_EQUAL, rhs
}

func getBendersCutOP(nodes []int, score int) (ind []int32, val []float64, op int8, rhs float64) {
	//priceSum := 0
	for i := 0; i < len(nodes); i++ {
		node := nodes[i]
		price := pInst.Prices[node]

		ind = append(ind, int32(node))
		val = append(val, float64(price))
		//priceSum += price
	}
	return ind, val, gurobi.LESS_EQUAL, float64(score)
}

func getSECs(subtours [][]int32) (secInd [][]int32, secVal [][]float64, op int8, rhs []float64) {
	for _, stour := range subtours {
		var (
			ind []int32
			val []float64
		)

		/* Add a subtour elimination constraint */
		for i := 0; i < len(stour); i++ {
			for j := i + 1; j < len(stour); j++ {
				if stour[j] > stour[i] {
					ind = append(ind, int32(getYIndex(int(stour[i]), int(stour[j]))))
				} else {
					ind = append(ind, int32(getYIndex(int(stour[j]), int(stour[i]))))
				}

			}
		}
		for i := 0; i < len(ind); i++ {
			val = append(val, 1.0)
		}

		secInd = append(secInd, ind)
		secVal = append(secVal, val)
		rhs = append(rhs, float64(len(stour)-1))
	}
	return secInd, secVal, gurobi.LESS_EQUAL, rhs
}

func masterCallback(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	myData := usrdata.(*MasterCallbackData)

	if where == gurobi.CB_MIPSOL {
		masterCbCount++
		solA, err := gurobi.CbGetDblArray(cbdata, where, gurobi.CB_MIPSOL_SOL, varCount)
		if err != nil {
			sol.Comment += fmt.Sprintf("Couldn't retrieve the array in the callback with the decision variables: %s. ", err.Error())
			log.Printf("At %s: %s\n", *inputF, sol.Comment)
			return 0
		}
		objval, err := gurobi.CbGetDbl(cbdata, where, gurobi.CB_MIPSOL_OBJ)
		if err != nil {
			sol.Comment += fmt.Sprintf("Couldn't retrieve the obj_value in the callback: %s. ", err.Error())
			log.Printf("At %s: %s\n", *inputF, sol.Comment)
			return 0
		}
		objVal := int(objval + 0.5)

		if int64(myData.CurrentSolObj+0.5) >= int64(objval+0.5) {
			log.Printf("Our current best solution %d is at least as good as the current master solution %d, so we do not solve the subproblem at this point", int64(myData.CurrentSolObj+0.5), int64(objval+0.5))
			return 0
		}

		var (
			heurObj        int
			heurSol        []int32
			heurTourLength int
			opTour         []int
			tspTour        []int32
			tspTourLength  int
			tspSubtours    [][]int32
			objSolValid    bool
		)

		//look for violated integer SECs first, before we solve the TSP. If we find any we add those first instead
		{
			edges := extractEdgeMatrix(solA)
			subSolNodes := extractNodeArray(solA)
			nodeCount := 0
			for i := 0; i < len(subSolNodes); i++ {
				if subSolNodes[i] > 0.5 { //Counting the nodes to be visited
					nodeCount++
				}
			}

			subtour := findIntSubtour(edges)
			if subtour != nil && len(subtour) < nodeCount {
				/*secInd, secVal, oper, rhs := op.GetSECs([][]int32{gurobi.Int32Slice(subtour)}, N, startY)
				for i := 0; i < len(secInd); i++ {
					log.Printf("Adding SEC in the callback for %d edges\n", len(secInd[i]))
					err = gurobi.CbLazy(cbdata, len(secInd[i]), secInd[i], secVal[i], oper, rhs[i])
					if err != nil {
						log.Println(err)
					}
				}
				return 0*/
			} else {
				//log.Printf("-------------------NO INTEGER-SEC VIOLATED!!!!!---------------------\n")
				if *yBounds == Y_BOUNDS_BIN || len(subtour) == nodeCount {
					//we don't need to check any further or solve the tsp
					heurSol = gurobi.Int32Slice(subtour)
					heurObj = objVal
					heurTourLength = op.GetTourLength(subtour, edgeDist)
					objSolValid = true
				}
			}
		}

		//we stop after the integer SECs for the LBBD strategy
		if *strat == LBBD {
			return 0
		}

		if !objSolValid {
			if *subStrat == OP {
				//no integer subtours found, we solve the op for the selected nodes to cutoff the solution
				xMat := extractNodeArray(solA)

				d, p, indx := transformToOP(xMat)
				opTour, heurObj, heurTourLength, _, _, err = op.SolveOP(d, p, pInst.TMax)

				//translate op tour to global indxs
				for k := 0; k < len(opTour); k++ {
					opTour[k] = indx[opTour[k]]
				}

				heurSol = gurobi.Int32Slice(opTour)
				activeNodes := extractActiveNodes(xMat)
				ind, val, op, rhs := getBendersCutOP(activeNodes, heurObj)
				// Add the benders cut
				err = gurobi.CbLazy(cbdata, len(ind), ind, val, op, rhs)
				if err != nil {
					log.Println(err)
				}
			} else if *subStrat == TSP {
				//no integer subtours found, we solve the tsp
				tspTour, tspTourLength, tspSubtours = solveSubproblem(solA)

				if tspTour != nil && tspTourLength > pInst.TMax {

					for i := 0; i < len(cuts); i++ {
						cut := cuts[i]
						log.Printf("The Master solution with obj %d cannot be correct - Adding a %s to cut it off\n", objVal, cut)
						if cut == SEC {
							if len(tspSubtours) > 0 {
								secInd, secVal, op, rhs := op.GetSECs(tspSubtours, N, startY)
								for j := 0; j < len(secInd); j++ {
									err = gurobi.CbLazy(cbdata, len(secInd[j]), secInd[j], secVal[j], op, rhs[j])
									if err != nil {
										log.Println(err)
									}
								}
							}
						}
						if cut == BEND_V0 {
							ind, val, op, rhs := getBendersCutV0(tspTour)
							// Add the benders cut
							err = gurobi.CbLazy(cbdata, len(ind), ind, val, op, rhs)
							if err != nil {
								log.Println(err)
							}
						}
						if cut == BEND_V1 {
							ind, val, op, rhs := getBendersCutV1(tspTour, tspTourLength)
							// Add the benders cut
							err = gurobi.CbLazy(cbdata, len(ind), ind, val, op, rhs)
							if err != nil {
								log.Println(err)
							}
							bendersCuts++
						}
						if cut == BEND_V2 {
							ind, val, op, rhs := getBendersCutV2(tspTour, tspTourLength, pInst.TMax)
							for i := 0; i < len(ind); i++ {
								// Add the benders cut
								err = gurobi.CbLazy(cbdata, len(ind[i]), ind[i], val[i], op, rhs[i])
								if err != nil {
									log.Println(err)
								}
								bendersCuts++
							}
						}
					}

					//calculate a heuristic tour with greedy strategy
					heurSol, heurTourLength, heurObj = shortenTour(tspTour, edgeDist, pInst.Prices, tspTourLength, pInst.TMax, objVal)
				} else {
					//the TSP-solution does not invalidate the master solution
					heurSol = tspTour
					heurObj = objVal
					heurTourLength = tspTourLength
				}
			}
		}

		//log.Printf("Current tour: %v\n", heurSol)
		if int(myData.CurrentSolObj) < heurObj {
			myData.CurrentSolObj = float64(heurObj)
			myData.NodeSequence = heurSol
			myData.TourLength = heurTourLength

			if objVal == heurObj {
				//The current master-solution has the same objval as the calculated sequences from ATSP, so the value has been used already before we get the chance to set the solution!
				log.Printf("The current master-solution has the same objval %d as the calculated sequence from TSP", heurObj)
				myData.NewBestSol = false
			} else if objVal > heurObj {
				//The heuristic solution is worse than the current objval, which means we added some benders cuts
				log.Printf("Found new best solution with value %d, while the master solution was invalid", heurObj)
				myData.NewBestSol = true
			} else {
				//The heuristic solution was better, than the master solution (this can happen??) HOW come??
				log.Printf("Found new best solution with value %d, which is even better than the current master solution!", heurObj)
				myData.NewBestSol = true
			}
		}

	}

	if where == gurobi.CB_MIPNODE {
		if myData.NewBestSol {
			objbst, err := gurobi.CbGetDbl(cbdata, where, gurobi.CB_MIPNODE_OBJBST)
			if err != nil {
				sol.Comment += fmt.Sprintf("Couldn't retrieve the obj_best in the callback: %s. ", err.Error())
				log.Printf("At %s: %s\n", *inputF, sol.Comment)
				return 0
			}
			if int(objbst+0.5) >= int(myData.CurrentSolObj+0.5) {
				log.Printf("Current obj is already better than the heuristic solution. Skipping...\n")
				myData.NewBestSol = false
				return 0
			}
			log.Printf("Currently setting new heuristic solution: %v with obj-value %.2f\n", myData.NodeSequence, myData.CurrentSolObj)
			solution := make([]float64, varCount)

			//set the objective (X_i values)
			for i := 0; i < len(myData.NodeSequence); i++ {
				solution[int32(startX)+myData.NodeSequence[i]] = 1.0
			}

			//set the constraints (Y_ij values)
			for i := 0; i < len(myData.NodeSequence); i++ {
				y := getYIndex(int(myData.NodeSequence[i]), int(myData.NodeSequence[(i+1)%len(myData.NodeSequence)]))
				solution[y] = 1.0
			}

			//set the solution
			val, err := gurobi.CbSolution(cbdata, solution)

			//check the error and objv
			if err != nil {
				log.Printf("Couldn't set the heuristic solution: %s\n", err.Error())
			} else {
				myData.NewBestSol = false
				log.Printf("New best solution with value : %d set!\n", int(val))
			}
		}
	}
	return 0
}

func shortenTour(tour []int32, edgeDist [][]int, prices []int, tourLength int, tmax int, tourObj int) ([]int32, int, int) {
	//oldTourLength := tourLength
	for tourLength > tmax {
		bestValRatio := 0.0
		bestValLoss := 0
		bestLengthGained := 0
		bestValAt := 1
		for j := 1; j < len(tour); j++ {
			i := j - 1
			k := (j + 1) % len(tour)
			lengthGain := edgeDist[tour[i]][tour[j]] + edgeDist[tour[j]][tour[k]] - edgeDist[tour[i]][tour[k]]
			valLoss := prices[tour[j]]
			valRatio := float64(lengthGain) / float64(valLoss)
			if valRatio > bestValRatio {
				bestValRatio = valRatio
				bestLengthGained = lengthGain
				bestValLoss = valLoss
				bestValAt = j
			}
		}
		tour = append(tour[:bestValAt], tour[bestValAt+1:]...)
		tourLength -= bestLengthGained
		tourObj -= bestValLoss
		//TODO: apply 3opt after each removal and check if the tour got shorter? (2opt can only remove crossing edges, which we cannot have?)
	}
	//log.Printf("Shortened the tour from %d to %d\n", oldTourLength, tourLength)
	//checkSolutionValidity(tour, edgeDist, prices, tmax, tourObj)
	return tour, tourLength, tourObj
}

func writeSolution() {
	jsonInst, err := json.MarshalIndent(pInst, "", "\t")
	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}
	jsonInst = []byte(op.SanitizeJsonArrayLineBreaks(string(jsonInst)))
	var fileName string
	if *outputF == "" {
		fileName = *inputF //overwrite the input file
	} else {
		fileName = *outputF //overwrite the input file
	}
	err = ioutil.WriteFile(fileName, jsonInst, 0644)
	if err != nil {
		log.Printf("At %s: %s\n", *inputF, err.Error())
		return
	}
}

func getYIndex(i, j int) int {
	return op.GetEdgeIndex(i, j, N, startY)
	/*if j < i {
		i, j = j, i
	}
	count := startY
	for k := 0; k < i; k++ {
		count += N0 - k
	}
	count += j - i - 1
	return count*/
}

func extractNodeArray(solA []float64) []float64 {
	return solA[startX:startY]
}

func extractActiveNodes(xMat []float64) []int {
	var activeNodes []int
	for i := 0; i < len(xMat); i++ {
		if xMat[i] > 0.5 {
			activeNodes = append(activeNodes, i)
		}
	}
	return activeNodes
}

func extractEdgeMatrix(solA []float64) [][]int {
	yMat := make([][]int, N)
	for i := 0; i < N; i++ {
		yMat[i] = make([]int, N)
	}
	for i := 0; i < N; i++ {
		for j := i + 1; j < N; j++ {
			if solA[getYIndex(i, j)] > 0.95 {
				//log.Printf("Rounding %.5f to 1\n",solA[getYIndex(i, j)])
				yMat[i][j] = 1
				yMat[j][i] = 1
			}
		}
	}
	return yMat
}
