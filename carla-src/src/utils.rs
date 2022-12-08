use anyhow::Result;
use sha2::{Digest, Sha256};
use std::{
    fs::{File, OpenOptions},
    io::{prelude::*, BufReader},
    path::Path,
};

pub fn sha256sum(path: &Path) -> Result<[u8; 32]> {
    let mut hasher = Sha256::new();
    let mut reader = BufReader::new(File::open(path)?);

    loop {
        let mut buf = [0u8; 8192];
        let len = reader.read(&mut buf)?;
        if len == 0 {
            break;
        }
        hasher.update(&buf[0..len]);
    }

    let hash = hasher.finalize();
    Ok(hash.as_slice().try_into().unwrap())
}

pub fn touch(path: &Path) -> Result<()> {
    OpenOptions::new().create(true).write(true).open(path)?;
    Ok(())
}

pub fn skip_or_run<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    with_target(target_path, || {
        let output = (callback)()?;
        touch(target_path)?;
        Ok(output)
    })
}

pub fn with_target<T, F>(target_path: &Path, callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    with_targets(&[target_path], callback)
}

pub fn with_targets<T, F>(target_paths: &[&Path], callback: F) -> Result<Option<T>>
where
    F: FnOnce() -> Result<T>,
{
    let ok = target_paths.iter().all(|path| path.exists());
    if ok {
        return Ok(None);
    }

    (callback)().map(Some)
}
