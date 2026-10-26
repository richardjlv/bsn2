"""Coleta isolada de cobertura dos pacotes e BDD, incluindo subprocessos Python."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import xml.etree.ElementTree as ET

PACKAGES = ("adaptation", "central_hub", "patient", "sensor", "shared_components", "system_monitor")
ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_OMIT = ["*/test/*", "*/tests/*", "*/test_components/*", "*/*_tests.py", "*/setup.py"]


def configuration(source_root, data_file, context):
    sources = "\n".join("    " + str(source_root / p / p) for p in PACKAGES)
    return ("[run]\nbranch = True\nparallel = True\nsigterm = True\n"
            f"data_file = {data_file}\ncontext = {context}\nsource =\n{sources}\n"
            "[report]\nshow_missing = True\nfail_under = 0\n")


def bootstrap_text(previous_sitecustomize):
    # Conserva o hook do ambiente, caso exista, sem alterar site-packages.
    return ("import os, runpy\n"
            f"previous = {previous_sitecustomize!r}\n"
            "if previous and os.path.isfile(previous):\n"
            "    runpy.run_path(previous)\n"
            "import coverage\ncoverage.process_startup()\n")


def run_logged(command, cwd, env, logfile):
    with logfile.open("w") as log:
        log.write(json.dumps({"command": command, "cwd": str(cwd)}) + "\n")
        log.flush()
        with subprocess.Popen(command, cwd=cwd, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True) as process:
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
            return process.wait()


def junit_counts(path):
    counts = Counter()
    cases = []
    root = ET.parse(path).getroot()
    if root.tag not in {"testsuite", "testsuites"}:
        raise ValueError(f"Arquivo não é JUnit: {path} (raiz {root.tag})")
    for case in root.iter("testcase"):
        skipped = case.find("skipped")
        state = ("error" if case.find("error") is not None else
                 "failure" if case.find("failure") is not None else
                 "xfail" if skipped is not None and skipped.get("type") == "pytest.xfail" else
                 "skip" if skipped is not None else "passed")
        counts[state] += 1
        cases.append({"name": case.get("name"), "classname": case.get("classname"),
                      "status": state, "skip_reason": skipped.attrib if skipped is not None else None})
    return {"path": str(path), "counts": dict(counts), "cases": cases}


def package_junit_reports(run):
    # XML de cobertura ficam no mesmo diretório; não são resultados pytest.
    paths = [run / "packages" / (package + ".xml") for package in PACKAGES]
    return [junit_counts(path) for path in paths if path.exists()]


def reports(config, destination, data_paths):
    from coverage import Coverage
    destination.mkdir(exist_ok=True)
    cov = Coverage(config_file=str(config), data_file=str(destination / ".coverage"))
    cov.combine(data_paths=[str(p) for p in data_paths], strict=True, keep=True)
    cov.save()
    for name, omit in (("raw", []), ("production", PRODUCTION_OMIT)):
        cov.html_report(directory=str(destination / ("html_" + name)), omit=omit)
        cov.xml_report(outfile=str(destination / ("coverage_" + name + ".xml")), omit=omit)
        cov.json_report(outfile=str(destination / ("coverage_" + name + ".json")), omit=omit)
    data = cov.get_data()
    return {"has_arcs": data.has_arcs(),
            "executed_files": [p for p in sorted(data.measured_files()) if data.lines(p)]}


def snapshot(destination):
    hashes = {}
    excluded = {".git", "build", "install", "log", "coverage_runs", "__pycache__", ".pytest_cache"}
    extensions = {".py", ".feature", ".yaml", ".yml", ".xml", ".ini", ".cfg", ".sh", ".msg", ".srv", ".action", ".toml"}
    with tarfile.open(destination / "sources.tar.gz", "w:gz") as archive:
        for current, dirs, files in os.walk(ROOT):
            dirs[:] = [d for d in dirs if d not in excluded and not d.startswith("htmlcov")]
            for name in files:
                path = Path(current) / name
                if path.suffix not in extensions and name != ".coveragerc":
                    continue
                relative = str(path.relative_to(ROOT))
                hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
                archive.add(path, arcname=relative, recursive=False)
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=ROOT / "coverage_runs")
    args = parser.parse_args()
    for name in ("coverage", "pytest", "rclpy", "launch_testing"):
        if importlib.util.find_spec(name) is None:
            parser.error(f"Dependência ausente: {name}. Carregue o ambiente ROS 2 e o workspace.")
    if "COVERAGE_PROCESS_START" in os.environ:
        parser.error("Já existe instrumentação COVERAGE_PROCESS_START no ambiente.")
    args.output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run = Path(tempfile.mkdtemp(prefix=f"run_{stamp}_", dir=args.output_root.resolve()))
    print(f"Coleta ROS 2: {run}", flush=True)
    bootstrap = run / "bootstrap"
    bootstrap.mkdir()
    previous = importlib.util.find_spec("sitecustomize")
    (bootstrap / "sitecustomize.py").write_text(bootstrap_text(previous.origin if previous else None))
    manifest = {"started_utc": stamp, "python": sys.version, "source_root": str(ROOT),
                "production_omit": PRODUCTION_OMIT, "source_sha256": snapshot(run),
                "commands": [], "reports": {}, "report_errors": [], "successful_collection": False}
    def save_manifest():
        (run / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    save_manifest()
    run_logged([sys.executable, "-m", "pip", "freeze"], ROOT, os.environ.copy(), run / "dependencies.log")
    failed = False
    phase_dirs = []
    for phase, packages in (("packages", PACKAGES), ("bdd", ("features",))):
        directory = run / phase
        directory.mkdir()
        phase_dirs.append(directory)
        config = directory / "coverage.ini"
        config.write_text(configuration(ROOT, directory / ".coverage", phase))
        env = dict(os.environ)
        env.pop("PYTEST_ADDOPTS", None)
        env.update(COVERAGE_PROCESS_START=str(config), COVERAGE_RCFILE=str(config),
                   COVERAGE_FILE=str(directory / ".coverage"), BSN_COVERAGE_CAMPAIGN="1")
        # Garante que entry points e subprocessos usem os módulos Python deste checkout.
        env["PYTHONPATH"] = os.pathsep.join([str(bootstrap), *(str(ROOT / p) for p in PACKAGES),
                                              env.get("PYTHONPATH", "")])
        if phase == "bdd":
            env["BSN_BDD_RESULTS_DIR"] = str(directory / "internal_junit")
        for package in packages:
            command = [sys.executable, "-m", "pytest", "-v", "-rA", "-o", "addopts=",
                       "-p", "no:pytest_cov", "--junitxml=" + str(directory / (package + ".xml")), "test"]
            code = run_logged(command, ROOT / package, env, directory / (package + ".log"))
            manifest["commands"].append({"phase": phase, "package": package, "command": command, "exit_code": code})
            failed |= code != 0
            save_manifest()
        try:
            manifest["reports"][phase] = reports(config, directory, [directory])
        except Exception as error:
            manifest["report_errors"].append({"phase": phase, "error": str(error)})
            failed = True
        save_manifest()
    try:
        manifest["reports"]["combined"] = reports(run / "bdd/coverage.ini", run / "combined", phase_dirs)
    except Exception as error:
        manifest["report_errors"].append({"phase": "combined", "error": str(error)})
        failed = True
    # Não somar os oito executores BDD aos cenários internos.
    manifest["package_junit"] = package_junit_reports(run)
    manifest["bdd_internal_junit"] = [junit_counts(p) for p in sorted((run / "bdd/internal_junit").glob("*.xml"))]
    outer = run / "bdd/features.xml"
    manifest["bdd_outer_junit"] = junit_counts(outer) if outer.exists() else None
    expected = {p.stem for p in (ROOT / "features/steps").glob("test_*.py")}
    found = {Path(p["path"]).stem for p in manifest["bdd_internal_junit"]}
    manifest["missing_bdd_reports"] = sorted(expected - found)
    failed |= bool(manifest["missing_bdd_reports"]) or len(manifest["package_junit"]) != len(PACKAGES)
    manifest["empty_junit_reports"] = [entry["path"] for entry in
                                       manifest["package_junit"] + manifest["bdd_internal_junit"]
                                       if not entry["cases"]]
    failed |= bool(manifest["empty_junit_reports"])
    manifest["successful_collection"] = not failed
    save_manifest()
    print(f"Relatórios e resultados: {run}\nEstado da coleta: {'com falhas/pendências' if failed else 'concluída'}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
