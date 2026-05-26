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
                        if (x[0] <= -0.3284243494272232) {
                            if (x[2] <= -0.28365400433540344) {
                                if (x[1] <= 0.021017253398895264) {
                                    return 1;
                                }

                                else {
                                    return 0;
                                }
                            }

                            else {
                                if (x[2] <= 0.0793040432035923) {
                                    return 0;
                                }

                                else {
                                    return 0;
                                }
                            }
                        }

                        else {
                            if (x[1] <= 0.5871562063694) {
                                if (x[2] <= 0.7085444182157516) {
                                    return 2;
                                }

                                else {
                                    if (x[2] <= 1.3168666362762451) {
                                        return 1;
                                    }

                                    else {
                                        return 1;
                                    }
                                }
                            }

                            else {
                                if (x[2] <= -1.039533942937851) {
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