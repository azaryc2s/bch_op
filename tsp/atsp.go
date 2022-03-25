/* Copyright 2021, Arkadiusz Zarychta */
/* Copyright 2021, Gurobi Optimization, LLC */

/*
  Solve a asymmetric traveling salesman problem. The base MIP model only includes
  'degree-2' constraints, requiring each node to have exactly
  two incident edges, one incoming and one outgoing.  Solutions to this model may contain subtours -
  tours that don't visit every node.  The lazy constraint callback
  adds new constraints to cut them off.
*/
package tsp

import (
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"log"
	"os"
	//"math"
	//"math/rand"
	//"os"
	//"strconv"
)

/* Given an integer-feasible solution 'sol', find the smallest
   sub-tour.  Result is returned in 'tour' */

func findsubtourATSP(sol [][]float64) (result []int32) {
	n := int32(len(sol))
	seen := make([]bool, n)
	tour := make([]int32, n)

	start := int32(0)
	bestlen := n + 1
	bestind := int32(-1)
	i := int32(0)
	node := int32(0)
	for start < n {
		for node = 0; node < n; node++ {
			if !seen[node] {
				break
			}
		}
		if node == n {
			break
		}
		for leng := int32(0); leng < n; leng++ {
			tour[start+leng] = node
			seen[node] = true
			for i = 0; i < n; i++ {
				if sol[node][i] > 0.5 && !seen[i] {
					node = i
					break
				}
			}
			if i == n {
				leng++
				if leng < bestlen {
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

func subtourelimATSP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	n := usrdata.(SubData).N

	if where == gurobi.CB_MIPSOL {
		sol, err := gurobi.CbGetDblMatrix(cbdata, where, gurobi.CB_MIPSOL_SOL, int(n))
		if err != nil {
			log.Println(err)
		}
		tour := findsubtourATSP(sol)
		if int32(len(tour)) < n {
			var (
				ind []int32
				val []float64
			)

			// Add a subtour elimination constraint
			for i := 0; i < len(tour); i++ {
				for j := 0; j < len(tour); j++ {
					ind = append(ind, tour[i]*n+tour[j])
				}
			}
			for i := 0; i < len(ind); i++ {
				val = append(val, 1.0)
			}

			err = gurobi.CbLazy(cbdata, len(ind), ind, val, gurobi.LESS_EQUAL, float64(len(tour)-1))
			if err != nil {
				log.Println(err)
			}
		}
	}

	return 0
}

func SolveATSP(d [][]int) ([]int32, int) {
	// Create environment
	var err error
	gurobiEnv, err = gurobi.LoadEnv("atsp_gurobi.log")
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return nil,-1
	}
	defer gurobiEnv.Free()

	gurobiEnv.SetIntParam("LogToConsole", int32(0))
	defer gurobiEnv.SetIntParam("LogToConsole", int32(1))
	n := len(d)

	/* Create an empty model */

	model, err := gurobiEnv.NewModel("atsp", 0, nil, nil, nil, nil, nil)
	if err != nil {
		log.Println(err)
		return nil, -1
	}
	defer model.Free()

	// Change objective sense to minimization
	err = model.SetIntAttr(gurobi.INT_ATTR_MODELSENSE, gurobi.MINIMIZE)
	if err != nil {
		log.Printf("At %s: %s\n", os.Args[1], err.Error())
		return nil, -1
	}

	/* Add variables - one for every pair of nodes */

	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			name := fmt.Sprintf("x_%d_%d", i, j)
			err = model.AddVar(nil, nil, float64(d[i][j]), 0.0, 1.0, gurobi.BINARY, name)
			if err != nil {
				log.Println(err)
				return nil, -1
			}
		}
	}

	/* Degree-2 constraints  - 1 -> and 1 <- */
	var (
		ind []int
		val []float64
	)

	for i := 0; i < n; i++ {
		ind = nil
		val = nil
		for j := 0; j < n; j++ {
			ind = append(ind, i*n+j)
			val = append(val, 1.0)
		}
		nameo := fmt.Sprintf("deg2o_%d", i)
		err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.EQUAL, 1, nameo)

		ind = nil
		val = nil
		for j := 0; j < n; j++ {
			ind = append(ind, j*n+i)
			val = append(val, 1.0)
		}
		namei := fmt.Sprintf("deg2i_%d", i)
		err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.EQUAL, 1, namei)
	}

	/* Forbid edge from node back to itself */
	for i := 0; i < n; i++ {
		err = model.SetDblAttrElem(gurobi.DBL_ATTR_UB, int32(i*n+i), 0)
		if err != nil {
			log.Println(err)
			return nil, -1
		}
	}

	/* Asymmetric TSP x[i][j] + x[j][i] <= 1 */
	/*
	ind = make([]int, 2)
	val = make([]float64, 2)
	count := 0
	for i := 0; i < n; i++ {
		for j := 0; j < i; j++ {
			ind[0] = i*n + j
			ind[1] = i + j*n
			val[0] = 1
			val[1] = 1
			err = model.AddConstr(gurobi.Int32Slice(ind), val, gurobi.LESS_EQUAL, 1.0, fmt.Sprintf("asym_%d", count))
			count++
			if err != nil {
				log.Println(err)
				return nil, -1
			}
		}
	}*/

	/* Set callback function */

	err = model.SetCallbackFuncGo(subtourelimATSP, SubData{N: int32(n)})
	if err != nil {
		log.Println(err)
		return nil, -1
	}

	/* Must set LazyConstraints parameter when using lazy constraints */

	err = model.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		log.Println(err)
		return nil, -1
	}

	/* Optimize model */

	err = model.Optimize()
	if err != nil {
		log.Println(err)
		return nil, -1
	}

	/* Extract solution */
	solcount, err := model.GetIntAttr(gurobi.INT_ATTR_SOLCOUNT)
	if err != nil {
		log.Println(err)
		return nil, -1
	}

	if solcount > 0 {
		solAtsp, err := model.GetDblAttrMatrix(gurobi.DBL_ATTR_X, 0, int32(n))
		if err != nil {
			log.Println(err)
			return nil, -1
		}
		tour := findsubtourATSP(solAtsp)
		length := 0
		for i := 0; i < len(tour)-1; i++ {
			length += d[int(tour[i])][int(tour[i+1])]
		}
		length += d[int(tour[len(tour)-1])][int(tour[0])]

		return tour, length
	}
	return nil, -1
}
