package main

import (
	"encoding/json"
	"errors"
	"fmt"
	"git.solver4all.com/azaryc2s/op"
	"io/ioutil"
	"log"
	"os"
	"strings"
)

func main() {
	if len(os.Args) < 2 {
		log.Printf("No arguments passed!")
		return
	}
	dirName := os.Args[1]
	dir, err := ioutil.ReadDir(dirName)
	if err != nil {
		log.Printf("Couldn't open directory %s: %s\n", os.Args[1], err.Error())
		return
	}
	fmt.Printf("Name,Optimal,Time,CMax_Obj,UBound,Gap,Dimension,Comment\n")
	for _, f := range dir {
		fileName := dirName + "/" + f.Name()
		if strings.Contains(fileName, ".json") {
			inst := op.Instance{}
			instStr, err := ioutil.ReadFile(fileName)
			if err != nil {
				log.Printf("Couldn't read %s: %s\n", f.Name(), err.Error())
				return
			}
			err = json.Unmarshal(instStr, &inst)
			if err != nil {
				log.Printf("Couldn't parse %s: %s\n", f.Name(), err.Error())
				return
			}
			var sol op.Solution
			if inst.Solution != nil {
				sol = *inst.Solution
			}
			_, err = getMaxSequenceLength(inst, sol)
			if err != nil {
				sol.Comment += fmt.Sprintf("ANALYZER: Error = %s", err.Error())
			}
			gap := 100.0 * (float64(sol.Obj-sol.UBound) / float64(sol.UBound))
			fmt.Printf("%s,%t,%s,%d,%d,%.4f,%d,%s\n", inst.Name, sol.Optimal, sol.Time, sol.Obj, sol.UBound, gap, inst.Dimension, sol.Comment)
		}
	}

}

func getMaxSequenceLength(inst op.Instance, sol op.Solution) (int, error) {
	sum := 0
	edgeWeights := op.CalcEdgeDist(inst.NodeCoordinates, inst.EdgeWeightType)
	used := make([]bool, inst.Dimension)
	for i := 0; i < len(sol.Route); i++ {
		a := sol.Route[i]
		b := sol.Route[(i+1)%len(sol.Route)]
		sum += edgeWeights[a][b]
		if used[a] {
			return -1, errors.New(fmt.Sprintf("Node %d visited twice!", a))
		}
		used[a] = true
	}
	if sum > inst.TMax {
		return -1, errors.New(fmt.Sprintf("Route length %d exceeds the max allowed length %d!", sum, inst.TMax))
	}
	return sum, nil
}
