#![allow(dead_code)]

use anyhow::Result;
use std::{collections::HashMap, fs::File, path::Path};

/// Statistics collection for performance monitoring
#[derive(Debug, Default, Clone)]
pub struct PerformanceStats {
    pub total_operations: u64,
    pub successful_operations: u64,
    pub failed_operations: u64,
    pub total_duration_ms: u64,
    pub min_duration_ms: Option<u64>,
    pub max_duration_ms: Option<u64>,
}

impl PerformanceStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_operation(&mut self, duration_ms: u64, success: bool) {
        self.total_operations += 1;
        self.total_duration_ms += duration_ms;

        if success {
            self.successful_operations += 1;
        } else {
            self.failed_operations += 1;
        }

        // Update min/max
        match self.min_duration_ms {
            None => self.min_duration_ms = Some(duration_ms),
            Some(min) if duration_ms < min => self.min_duration_ms = Some(duration_ms),
            _ => {}
        }

        match self.max_duration_ms {
            None => self.max_duration_ms = Some(duration_ms),
            Some(max) if duration_ms > max => self.max_duration_ms = Some(duration_ms),
            _ => {}
        }
    }

    pub fn success_rate(&self) -> f64 {
        if self.total_operations == 0 {
            0.0
        } else {
            self.successful_operations as f64 / self.total_operations as f64
        }
    }

    pub fn average_duration_ms(&self) -> f64 {
        if self.total_operations == 0 {
            0.0
        } else {
            self.total_duration_ms as f64 / self.total_operations as f64
        }
    }
}

/// Simple CSV writer for exporting data
pub struct CsvWriter {
    writer: csv::Writer<File>,
}

impl CsvWriter {
    pub fn new<P: AsRef<Path>>(path: P) -> Result<Self> {
        let file = File::create(path)?;
        let writer = csv::Writer::from_writer(file);
        Ok(Self { writer })
    }

    pub fn write_header(&mut self, headers: &[&str]) -> Result<()> {
        self.writer.write_record(headers)?;
        Ok(())
    }

    pub fn write_row(&mut self, values: &[String]) -> Result<()> {
        self.writer.write_record(values)?;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }
}

/// Progress tracker for long-running operations
pub struct ProgressTracker {
    total: u64,
    current: u64,
    last_printed: u64,
    print_interval: u64,
}

impl ProgressTracker {
    pub fn new(total: u64, print_interval: u64) -> Self {
        Self {
            total,
            current: 0,
            last_printed: 0,
            print_interval,
        }
    }

    pub fn update(&mut self, current: u64) {
        self.current = current;

        if current - self.last_printed >= self.print_interval || current == self.total {
            let percentage = if self.total > 0 {
                (current as f64 / self.total as f64) * 100.0
            } else {
                0.0
            };

            println!("Progress: {}/{} ({:.1}%)", current, self.total, percentage);
            self.last_printed = current;
        }
    }

    pub fn increment(&mut self) {
        self.update(self.current + 1);
    }

    pub fn is_complete(&self) -> bool {
        self.current >= self.total
    }
}

// Configuration loader for JSON files
// NOTE: Commented out as serde is not available in examples
// pub fn load_json_config<T: serde::de::DeserializeOwned, P: AsRef<Path>>(path: P) -> Result<T> {
//     let content = std::fs::read_to_string(path)?;
//     let config: T = serde_json::from_str(&content)?;
//     Ok(config)
// }
//
// /// Save JSON configuration to file
// pub fn save_json_config<T: serde::Serialize, P: AsRef<Path>>(config: &T, path: P) -> Result<()> {
//     let content = serde_json::to_string_pretty(config)?;
//     std::fs::write(path, content)?;
//     Ok(())
// }

/// Simple table formatter for console output
pub struct TableFormatter {
    headers: Vec<String>,
    rows: Vec<Vec<String>>,
    max_widths: Vec<usize>,
}

impl TableFormatter {
    pub fn new(headers: Vec<String>) -> Self {
        let max_widths = headers.iter().map(|h| h.len()).collect();
        Self {
            headers,
            rows: Vec::new(),
            max_widths,
        }
    }

    pub fn add_row(&mut self, row: Vec<String>) {
        // Update max widths
        for (i, cell) in row.iter().enumerate() {
            if i < self.max_widths.len() {
                self.max_widths[i] = self.max_widths[i].max(cell.len());
            }
        }
        self.rows.push(row);
    }

    pub fn print(&self) {
        // Print header
        self.print_separator();
        self.print_row(&self.headers);
        self.print_separator();

        // Print rows
        for row in &self.rows {
            self.print_row(row);
        }
        self.print_separator();
    }

    fn print_row(&self, row: &[String]) {
        print!("|");
        for (i, cell) in row.iter().enumerate() {
            let width = self.max_widths.get(i).unwrap_or(&10);
            print!(" {cell:width$} |");
        }
        println!();
    }

    fn print_separator(&self) {
        print!("+");
        for width in &self.max_widths {
            print!("{}", "-".repeat(width + 2));
            print!("+");
        }
        println!();
    }
}

/// Simple histogram for data analysis
#[derive(Debug)]
pub struct Histogram {
    bins: HashMap<String, u64>,
    total_count: u64,
}

impl Histogram {
    pub fn new() -> Self {
        Self {
            bins: HashMap::new(),
            total_count: 0,
        }
    }

    pub fn add(&mut self, key: String) {
        *self.bins.entry(key).or_insert(0) += 1;
        self.total_count += 1;
    }

    pub fn print(&self) {
        println!("Histogram (total: {}):", self.total_count);
        let mut items: Vec<_> = self.bins.iter().collect();
        items.sort_by(|a, b| b.1.cmp(a.1)); // Sort by count descending

        for (key, count) in items {
            let percentage = if self.total_count > 0 {
                (*count as f64 / self.total_count as f64) * 100.0
            } else {
                0.0
            };
            println!("  {key}: {count} ({percentage:.1}%)");
        }
    }

    pub fn get_total(&self) -> u64 {
        self.total_count
    }

    pub fn get_count(&self, key: &str) -> u64 {
        *self.bins.get(key).unwrap_or(&0)
    }
}

impl Default for Histogram {
    fn default() -> Self {
        Self::new()
    }
}

/// Timer for measuring execution time
pub struct Timer {
    start: std::time::Instant,
}

impl Timer {
    pub fn new() -> Self {
        Self {
            start: std::time::Instant::now(),
        }
    }

    pub fn elapsed_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }

    pub fn reset(&mut self) {
        self.start = std::time::Instant::now();
    }
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}
