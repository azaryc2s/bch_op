package op

import (
	"fmt"
	"strconv"
)

type ArrayStringFlags []string

func (i *ArrayStringFlags) String() string {
	return fmt.Sprintf("%v",*i)
}

func (i *ArrayStringFlags) Set(value string) error {
	*i = append(*i, value)
	return nil
}

type ArrayIntFlags []int

func (i *ArrayIntFlags) String() string {
	return fmt.Sprintf("%v",*i)
}

func (i *ArrayIntFlags) Set(value string) error {
	val, err := strconv.Atoi(value)
	if err != nil {
		return err
	}
	*i = append(*i, val)
	return nil
}

type ArrayFloatFlags []float64

func (i *ArrayFloatFlags) String() string {
	return fmt.Sprintf("%v",*i)
}

func (i *ArrayFloatFlags) Set(value string) error {
	val, err := strconv.ParseFloat(value,64)
	if err != nil {
		return err
	}
	*i = append(*i, val)
	return nil
}