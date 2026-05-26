#pragma once
#include <cstdarg>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class PlantClassifier {
                public:
                    /**
                    * Predict class for features vector
                    */
                    int predict(float *x) {
                        if (x[0] <= -0.329419270157814) {
                            if (x[2] <= -0.24989820271730423) {
                                if (x[1] <= 0.020659327507019043) {
                                    return 1;
                                }

                                else {
                                    return 0;
                                }
                            }

                            else {
                                if (x[2] <= 0.1588461548089981) {
                                    return 0;
                                }

                                else {
                                    return 0;
                                }
                            }
                        }

                        else {
                            if (x[1] <= 0.5869736969470978) {
                                if (x[2] <= 0.867463730275631) {
                                    return 2;
                                }

                                else {
                                    if (x[2] <= 1.5525243878364563) {
                                        return 1;
                                    }

                                    else {
                                        return 1;
                                    }
                                }
                            }

                            else {
                                if (x[2] <= -1.1011306643486023) {
                                    return 2;
                                }

                                else {
                                    return 1;
                                }
                            }
                        }
                    }

                protected:
                };
            }
        }
    }