use diff_drive::rigid2d::Vector2D;
use diff_drive::utils::almost_equal;
use diff_drive::utils::distance;

/// Cluster object contains a vector of points (Vector2D)
/// which are points that are all within some linear distance of each other
/// specified by the given tolerance
#[derive(Debug, Clone)]
pub struct Cluster {
    threshold: f64,
    points: Vec<Vector2D<f64>>,
}

impl Cluster {
    /// Creates a new cluster object with the given distance tolerance
    pub fn new(threshold: f64) -> Self {
        Cluster {
            threshold,
            points: vec![],
        }
    }

    /// Creates a new cluster object with the given distance tolerance
    /// and adds the given point to the cluster
    pub fn new_with_point(threshold: f64, point: Vector2D<f64>) -> Self {
        Cluster {
            threshold,
            points: vec![point],
        }
    }

    /// Returns true if the cluster contains the point (within some tolerance)
    /// otherwise returns false. Currently the tolerance is hardcoded for almost_equal
    pub fn contains(&self, point: Vector2D<f64>) -> bool {
        for p in &self.points {
            if almost_equal(p.x, point.x, 1e-6) && almost_equal(p.y, point.y, 1e-6) {
                return true;
            }
        }
        return false;
    }

    /// Adds a point to the cluster without checking first that it
    /// belongs (is within the threshold distance)
    pub fn blind_add(&mut self, point: Vector2D<f64>) -> bool {
        if self.contains(point) {
            return false;
        } else {
            self.points.push(point);
            return true;
        }
    }

    /// Adds a point to the cluster if it is within the
    /// threshold distance of some other point in the cluster
    pub fn add(&mut self, point: Vector2D<f64>) -> bool {
        if self.points.len() == 0 {
            self.points.push(point);
            return true;
        }

        for p in &self.points {
            if !self.contains(point) {
                let d = distance(*p, point);
                if d > 0.0 && d < self.threshold {
                    self.points.push(point);
                    return true;
                }
            } else {
                break;
            }
        }
        return false;
    }

    /// Returns the vector of points in the cluster
    pub fn to_vec(&self) -> Vec<Vector2D<f64>> {
        self.points.clone()
    }
}

/// Given a vector of points, group all the points
/// into clusters with the given distance threshold
pub fn clusterize(points: &Vec<Vector2D<f64>>, threshold: f64) -> Vec<Cluster> {
    let mut all_clusters = vec![Cluster::new(threshold)];
    for p in points {
        let mut added = false;
        for cluster in &mut all_clusters {
            added = cluster.add(*p);
            if added {
                break;
            }
        }
        if !added {
            all_clusters.push(Cluster::new_with_point(threshold, *p));
        }
    }
    all_clusters
}

/// Gets the distance threshold to use for a cluster
/// this is pretty dumb. By doing this, we basically are
/// assuming the first two points are on the same curve,
/// AKA in the same cluster. This seems to mostly work...
pub fn get_threshold(points: &Vec<Vector2D<f64>>) -> f64 {
    let d = distance(points[0], points[1]);
    d + d * 0.05
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new() {
        let cluster = Cluster::new(1.0);
        assert!(almost_equal(cluster.threshold, 1.0, 1e-9));
    }

    #[test]
    fn new_from_point() {
        let cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        assert!(almost_equal(cluster.threshold, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].x, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].y, 2.0, 1e-6));
    }

    #[test]
    fn contains() {
        let cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        assert!(cluster.contains(Vector2D { x: 1.0, y: 2.0 }));
    }

    #[test]
    fn blind_add() {
        let mut cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        let should_be_true = cluster.blind_add(Vector2D { x: 2.0, y: 3.0 });
        assert!(should_be_true);

        let should_be_false = cluster.blind_add(Vector2D { x: 1.0, y: 2.0 });
        assert!(!should_be_false);

        assert_eq!(cluster.points.len(), 2);
        assert!(almost_equal(cluster.points[0].x, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].y, 2.0, 1e-6));
        assert!(almost_equal(cluster.points[1].x, 2.0, 1e-6));
        assert!(almost_equal(cluster.points[1].y, 3.0, 1e-6));
    }

    #[test]
    fn add() {
        let mut cluster = Cluster::new(1.0);
        cluster.add(Vector2D { x: 1.0, y: 0.0 });
        assert!(almost_equal(cluster.points[0].x, 1.0, 1e-6));
        assert!(almost_equal(cluster.points[0].y, 0.0, 1e-6));

        cluster.add(Vector2D { x: 1.5, y: 0.0 });
        assert!(almost_equal(cluster.points[1].x, 1.5, 1e-6));
        assert!(almost_equal(cluster.points[1].y, 0.0, 1e-6));

        cluster.add(Vector2D { x: 2.6, y: 0.0 });
        assert_eq!(cluster.points.len(), 2);
    }

    #[test]
    fn to_vec() {
        let cluster = Cluster::new_with_point(1.0, Vector2D { x: 1.0, y: 2.0 });
        let v = cluster.to_vec();
        assert!(almost_equal(v[0].x, 1.0, 1e-6));
        assert!(almost_equal(v[0].y, 2.0, 1e-6));
    }

    #[test]
    fn cluster_from_csv() {
        fn read_csv_trajectory(csv_path: &str) -> anyhow::Result<Vec<Vector2D<f64>>> {
            let mut reader = csv::Reader::from_path(csv_path)?;

            let mut points: Vec<Vector2D<f64>> = vec![];

            // note this skips the first line of the csv file
            for res in reader.records() {
                let record = res?;
                let x: f64 = record[0].parse().unwrap();
                let y: f64 = record[1].parse().unwrap();
                let v = Vector2D::new(x, y);
                points.push(v);
            }

            Ok(points)
        }

        // Get points along target trajectory
        // Path is hardcoded, will fail if file is not there
        let points: Vec<Vector2D<f64>> =
            read_csv_trajectory("/home/nick/GitHub/VanGo/proto/rocket.csv")
                .expect("File not found - note path is hardcoded in the test");

        // Cluster the points by the threshold distance
        // const THRESHOLD: f64 = 10.0;
        let threshold: f64 = get_threshold(&points);
        println!("threshold = {}", threshold);
        let all_clusters = clusterize(&points, threshold);
        println!("\n{} total clusters", all_clusters.len());

        // write all the clustered points to a csv file
        let mut csv_file = csv::Writer::from_path("/home/nick/GitHub/VanGo/proto/cluster_out.csv")
            .expect("couldn't create csv file");

        let mut cluster_count = 0.0; // this tells which cluster the point is a part of
        for cluster in &all_clusters {
            let cluster_vec = cluster.clone().to_vec();
            for point in &cluster.to_vec() {
                let line = [point.x, point.y, cluster_count].map(|e| e.to_string());
                csv_file.write_record(line).expect("failed to write line");
            }
            cluster_count += 1.0;
        }
    }
}
