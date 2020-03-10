
    //far divide
    /*double min_dist_2 = 1.0 / 0.0; //inf
    double min_t_begin = 0;
    double min_t_end = 0;
    double min_t_dt = 0;

    for(int segment = 0; segment < spline_max_t_; segment++) {
        //get spline len
        double segment_arclen = lengthOfSpline(segment, segment + 1);
        double segment_dt = density_far / segment_arclen;

        //sample points
        for(double t = segment; t < segment + 1; t += segment_dt) {
            double dx = x - spline_x_(t);
            double dy = y - spline_y_(t);
            double dist_2 = dx * dx + dy * dy;
            
            if(dist_2 < min_dist_2) {
                min_dist_2 = dist_2;

                min_t_begin = std::max(double(segment),     t - segment_dt);
                min_t_end   = std::min(double(segment + 1), t + segment_dt);
                min_t_dt    = density_near / segment_arclen;
            }
        }
    }

    //near divide
    double near_min_dist_2 = 1.0 / 0.0; //inf
    double near_min_t = 0;

    for(double t = min_t_begin; t < min_t_end; t += min_t_dt) {
        double dx = x - spline_x_(t);
        double dy = y - spline_y_(t);
        double dist_2 = dx * dx + dy * dy;

        if(dist_2 < near_min_dist_2) {
            near_min_dist_2 = dist_2;
            near_min_t = t;
        }
    }

    return near_min_t;*/