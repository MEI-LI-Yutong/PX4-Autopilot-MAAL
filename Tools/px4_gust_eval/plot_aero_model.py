#!/usr/bin/env python3
"""
Aerodynamic Model Visualization Tool

Reads LiftDrag and AdvancedLiftDrag plugin parameters from Gazebo SDF files
and plots aerodynamic coefficients (CL, CD, Cm, Cl, Cn, CY) vs angle of attack
and sideslip angle.

Usage:
    uv run Tools/px4_gust_eval/plot_aero_model.py \
        Tools/simulation/gz/models/tiltrotor/model.sdf \
        --output plots/aero_tiltrotor
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict

import matplotlib.pyplot as plt
import numpy as np

try:
    import scienceplots  # noqa: F401
    plt.style.use(["science", "no-latex"])
except Exception:
    pass


# ============================================================================
# SDF Parameter Parser
# ============================================================================

def parse_sdf_file(sdf_path: Path) -> Dict:
    """Parse SDF file and extract first LiftDrag or AdvancedLiftDrag plugin parameters."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Find all plugin elements
    for plugin in root.iter('plugin'):
        plugin_name = plugin.get('filename', '')

        if 'advanced-lift-drag' in plugin_name or 'AdvancedLiftDrag' in plugin_name:
            return parse_advanced_liftdrag(plugin)
        elif 'lift-drag' in plugin_name or 'LiftDrag' in plugin_name:
            return parse_liftdrag(plugin)

    raise ValueError(f"No LiftDrag or AdvancedLiftDrag plugin found in {sdf_path}")


def parse_liftdrag(plugin_elem: ET.Element) -> Dict:
    """Parse simple LiftDrag plugin parameters."""
    def get_float(name: str, default: float = 0.0) -> float:
        elem = plugin_elem.find(name)
        return float(elem.text) if elem is not None else default

    params = {
        'model_type': 'LiftDrag',
        'a0': get_float('a0', 0.0),
        'cla': get_float('cla', 1.0),
        'cda': get_float('cda', 0.01),
        'cma': get_float('cma', 0.0),
        'alpha_stall': get_float('alpha_stall', np.pi/2),
        'cla_stall': get_float('cla_stall', 0.0),
        'cda_stall': get_float('cda_stall', 1.0),
        'cma_stall': get_float('cma_stall', 0.0),
        'area': get_float('area', 1.0),
        'air_density': get_float('air_density', 1.2041),
    }
    return params


def parse_advanced_liftdrag(plugin_elem: ET.Element) -> Dict:
    """Parse AdvancedLiftDrag plugin parameters."""
    def get_float(name: str, default: float = 0.0) -> float:
        elem = plugin_elem.find(name)
        return float(elem.text) if elem is not None else default

    params = {
        'model_type': 'AdvancedLiftDrag',
        # Zero-alpha coefficients
        'a0': get_float('a0', 0.0),
        'CL0': get_float('CL0', 0.0),
        'CD0': get_float('CD0', 0.0),
        'Cem0': get_float('Cem0', 0.0),

        # Alpha derivatives
        'CLa': get_float('CLa', 0.0),
        'CYa': get_float('CYa', 0.0),
        'Cella': get_float('Cella', 0.0),
        'Cema': get_float('Cema', 0.0),
        'Cena': get_float('Cena', 0.0),

        # Beta derivatives
        'CLb': get_float('CLb', 0.0),
        'CYb': get_float('CYb', 0.0),
        'Cellb': get_float('Cellb', 0.0),
        'Cemb': get_float('Cemb', 0.0),
        'Cenb': get_float('Cenb', 0.0),

        # Stall parameters
        'alpha_stall': get_float('alpha_stall', np.pi/2),
        'CLa_stall': get_float('CLa_stall', 0.0),
        'CDa_stall': get_float('CDa_stall', 0.0),
        'Cema_stall': get_float('Cema_stall', 0.0),

        # Geometry
        'AR': get_float('AR', 1.0),
        'eff': get_float('eff', 0.9),
        'area': get_float('area', 1.0),
        'mac': get_float('mac', 0.0),
        'air_density': get_float('air_density', 1.2041),

        # Rate derivatives (p, q, r) - included but not plotted
        'CDp': get_float('CDp', 0.0),
        'CYp': get_float('CYp', 0.0),
        'CLp': get_float('CLp', 0.0),
        'Cellp': get_float('Cellp', 0.0),
        'Cemp': get_float('Cemp', 0.0),
        'Cenp': get_float('Cenp', 0.0),

        'CDq': get_float('CDq', 0.0),
        'CYq': get_float('CYq', 0.0),
        'CLq': get_float('CLq', 0.0),
        'Cellq': get_float('Cellq', 0.0),
        'Cemq': get_float('Cemq', 0.0),
        'Cenq': get_float('Cenq', 0.0),

        'CDr': get_float('CDr', 0.0),
        'CYr': get_float('CYr', 0.0),
        'CLr': get_float('CLr', 0.0),
        'Cellr': get_float('Cellr', 0.0),
        'Cemr': get_float('Cemr', 0.0),
        'Cenr': get_float('Cenr', 0.0),
    }

    # Blending parameters
    params['M'] = 15.0  # Sigmoid blending steepness
    params['CD_fp_k1'] = -0.224  # Flat plate drag coeff
    params['CD_fp_k2'] = -0.115

    return params


# ============================================================================
# Aerodynamic Model Implementations
# ============================================================================

class LiftDragModel:
    """Simple LiftDrag model implementation."""

    def __init__(self, params: Dict):
        self.params = params

    def CL(self, alpha: np.ndarray, beta: np.ndarray = None) -> np.ndarray:
        """Lift coefficient vs angle of attack."""
        alpha = np.asarray(alpha)
        a0 = self.params['a0']
        alpha_eff = alpha - a0
        alpha_stall = self.params['alpha_stall']

        CL = np.zeros_like(alpha)

        # Pre-stall region
        mask_pre = np.abs(alpha_eff) <= alpha_stall
        CL[mask_pre] = self.params['cla'] * alpha_eff[mask_pre]

        # Post-stall region (positive alpha)
        mask_post_pos = alpha_eff > alpha_stall
        CL[mask_post_pos] = (
            self.params['cla'] * alpha_stall +
            self.params['cla_stall'] * (alpha_eff[mask_post_pos] - alpha_stall)
        )

        # Post-stall region (negative alpha)
        mask_post_neg = alpha_eff < -alpha_stall
        CL[mask_post_neg] = (
            -self.params['cla'] * alpha_stall +
            self.params['cla_stall'] * (alpha_eff[mask_post_neg] + alpha_stall)
        )

        return CL

    def CD(self, alpha: np.ndarray, beta: np.ndarray = None) -> np.ndarray:
        """Drag coefficient vs angle of attack."""
        alpha = np.asarray(alpha)
        a0 = self.params['a0']
        alpha_eff = alpha - a0
        alpha_stall = self.params['alpha_stall']

        CD = np.zeros_like(alpha)

        # Pre-stall region
        mask_pre = np.abs(alpha_eff) <= alpha_stall
        CD[mask_pre] = self.params['cda'] * np.abs(alpha_eff[mask_pre])

        # Post-stall region
        mask_post_pos = alpha_eff > alpha_stall
        CD[mask_post_pos] = (
            self.params['cda'] * alpha_stall +
            self.params['cda_stall'] * (alpha_eff[mask_post_pos] - alpha_stall)
        )

        mask_post_neg = alpha_eff < -alpha_stall
        CD[mask_post_neg] = (
            -self.params['cda'] * alpha_stall +
            self.params['cda_stall'] * (alpha_eff[mask_post_neg] + alpha_stall)
        )

        return np.abs(CD)

    def Cm(self, alpha: np.ndarray, beta: np.ndarray = None) -> np.ndarray:
        """Pitching moment coefficient vs angle of attack."""
        alpha = np.asarray(alpha)
        a0 = self.params['a0']
        alpha_eff = alpha - a0
        alpha_stall = self.params['alpha_stall']

        Cm = np.zeros_like(alpha)

        # Pre-stall region
        mask_pre = np.abs(alpha_eff) <= alpha_stall
        Cm[mask_pre] = self.params['cma'] * alpha_eff[mask_pre]

        # Post-stall region
        mask_post_pos = alpha_eff > alpha_stall
        Cm[mask_post_pos] = (
            self.params['cma'] * alpha_stall +
            self.params['cma_stall'] * (alpha_eff[mask_post_pos] - alpha_stall)
        )

        mask_post_neg = alpha_eff < -alpha_stall
        Cm[mask_post_neg] = (
            -self.params['cma'] * alpha_stall +
            self.params['cma_stall'] * (alpha_eff[mask_post_neg] + alpha_stall)
        )

        return Cm


class AdvancedLiftDragModel:
    """Advanced LiftDrag model with stability derivatives."""

    def __init__(self, params: Dict):
        self.params = params

    def _sigmoid(self, alpha: np.ndarray) -> np.ndarray:
        """Sigmoid blending function for pre/post-stall."""
        M = self.params['M']
        alpha_stall = self.params['alpha_stall']

        # Avoid overflow in exp
        arg1 = np.clip(-M * (alpha - alpha_stall), -50, 50)
        arg2 = np.clip(M * (alpha + alpha_stall), -50, 50)

        sigma = (
            (1 + np.exp(arg1) + np.exp(arg2)) /
            ((1 + np.exp(arg1)) * (1 + np.exp(arg2)))
        )
        return sigma

    def CL(self, alpha: np.ndarray, beta: np.ndarray = None) -> np.ndarray:
        """Lift coefficient vs angle of attack."""
        alpha = np.asarray(alpha)
        if beta is None:
            beta = np.zeros_like(alpha)
        else:
            beta = np.asarray(beta)

        sigma = self._sigmoid(alpha)

        # Pre-stall model
        CL_pre = self.params['CL0'] + self.params['CLa'] * alpha

        # Post-stall model
        sin_alpha = np.sin(alpha)
        cos_alpha = np.cos(alpha)
        CL_post = 2 * np.sign(alpha) * sin_alpha**2 * cos_alpha

        # Blend
        CL = (1 - sigma) * CL_pre + sigma * CL_post

        # Add sideslip effect
        CL += self.params['CLb'] * beta

        return CL

    def CD(self, alpha: np.ndarray, beta: np.ndarray = None) -> np.ndarray:
        """Drag coefficient vs angle of attack."""
        alpha = np.asarray(alpha)
        sigma = self._sigmoid(alpha)

        # Pre-stall: induced drag model
        CL = self.CL(alpha, beta)
        AR = self.params['AR']
        eff = self.params['eff']
        CD_pre = self.params['CD0'] + (CL**2) / (np.pi * AR * eff)

        # Post-stall: flat plate drag
        AR_eff = max(AR, 1.0/AR)
        CD_fp = 2 / (1 + np.exp(self.params['CD_fp_k1'] +
                                 self.params['CD_fp_k2'] * AR_eff))
        CD_post = np.abs(CD_fp * (0.5 - 0.5 * np.cos(2 * alpha)))

        # Blend
        CD = (1 - sigma) * CD_pre + sigma * CD_post

        return CD

    def Cm(self, alpha: np.ndarray, beta: np.ndarray = None) -> np.ndarray:
        """Pitching moment coefficient vs angle of attack."""
        alpha = np.asarray(alpha)
        if beta is None:
            beta = np.zeros_like(alpha)
        else:
            beta = np.asarray(beta)

        alpha_stall = self.params['alpha_stall']
        Cm = np.zeros_like(alpha)

        # Post-stall positive alpha
        mask_pos = alpha > alpha_stall
        Cm[mask_pos] = (
            self.params['Cem0'] +
            self.params['Cema'] * alpha_stall +
            self.params['Cema_stall'] * (alpha[mask_pos] - alpha_stall)
        )

        # Post-stall negative alpha
        mask_neg = alpha < -alpha_stall
        Cm[mask_neg] = (
            self.params['Cem0'] +
            -self.params['Cema'] * alpha_stall +
            self.params['Cema_stall'] * (alpha[mask_neg] + alpha_stall)
        )

        # Pre-stall
        mask_pre = ~(mask_pos | mask_neg)
        Cm[mask_pre] = self.params['Cem0'] + self.params['Cema'] * alpha[mask_pre]

        # Add sideslip effect
        Cm += self.params['Cemb'] * beta

        return Cm

    def Cl(self, alpha: np.ndarray, beta: np.ndarray) -> np.ndarray:
        """Roll moment coefficient (primarily from sideslip)."""
        alpha = np.asarray(alpha)
        beta = np.asarray(beta)
        return self.params['Cella'] * alpha + self.params['Cellb'] * beta

    def Cn(self, alpha: np.ndarray, beta: np.ndarray) -> np.ndarray:
        """Yaw moment coefficient (primarily from sideslip)."""
        alpha = np.asarray(alpha)
        beta = np.asarray(beta)
        return self.params['Cena'] * alpha + self.params['Cenb'] * beta

    def CY(self, alpha: np.ndarray, beta: np.ndarray) -> np.ndarray:
        """Side force coefficient (primarily from sideslip)."""
        alpha = np.asarray(alpha)
        beta = np.asarray(beta)
        return self.params['CYa'] * alpha + self.params['CYb'] * beta


# ============================================================================
# Plotting Functions
# ============================================================================

def plot_coefficients_vs_alpha(model, output_dir: Path, model_name: str):
    """Plot aerodynamic coefficients vs angle of attack."""
    alpha_deg = np.linspace(-30, 30, 200)
    alpha_rad = np.deg2rad(alpha_deg)

    CL = model.CL(alpha_rad)
    CD = model.CD(alpha_rad)
    Cm = model.Cm(alpha_rad)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # CL vs alpha
    axes[0].plot(alpha_deg, CL, 'b-', linewidth=2)
    axes[0].axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[0].axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[0].set_xlabel('Angle of Attack (deg)', fontsize=12)
    axes[0].set_ylabel('$C_L$', fontsize=12)
    axes[0].set_title('Lift Coefficient', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)

    # Mark stall angle
    if 'alpha_stall' in model.params:
        alpha_stall_deg = np.rad2deg(model.params['alpha_stall'])
        axes[0].axvline(alpha_stall_deg, color='r', linestyle=':',
                      linewidth=1.5, label=f'Stall: {alpha_stall_deg:.1f}°')
        axes[0].axvline(-alpha_stall_deg, color='r', linestyle=':', linewidth=1.5)
        axes[0].legend()

    # CD vs alpha
    axes[1].plot(alpha_deg, CD, 'r-', linewidth=2)
    axes[1].axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[1].axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[1].set_xlabel('Angle of Attack (deg)', fontsize=12)
    axes[1].set_ylabel('$C_D$', fontsize=12)
    axes[1].set_title('Drag Coefficient', fontsize=14, fontweight='bold')
    axes[1].grid(True, alpha=0.3)

    # Cm vs alpha
    axes[2].plot(alpha_deg, Cm, 'g-', linewidth=2)
    axes[2].axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[2].axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[2].set_xlabel('Angle of Attack (deg)', fontsize=12)
    axes[2].set_ylabel('$C_m$', fontsize=12)
    axes[2].set_title('Pitching Moment Coefficient', fontsize=14, fontweight='bold')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = output_dir / f'{model_name}_coeffs_vs_alpha.png'
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")


def plot_coefficients_vs_beta(model, output_dir: Path, model_name: str):
    """Plot aerodynamic coefficients vs sideslip angle (for AdvancedLiftDrag)."""
    if not isinstance(model, AdvancedLiftDragModel):
        return  # Only for advanced model

    beta_deg = np.linspace(-20, 20, 200)
    beta_rad = np.deg2rad(beta_deg)
    alpha_rad = np.zeros_like(beta_rad)  # Zero AoA

    CY = model.CY(alpha_rad, beta_rad)
    Cl = model.Cl(alpha_rad, beta_rad)
    Cn = model.Cn(alpha_rad, beta_rad)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # CY vs beta
    axes[0].plot(beta_deg, CY, 'b-', linewidth=2)
    axes[0].axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[0].axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[0].set_xlabel('Sideslip Angle (deg)', fontsize=12)
    axes[0].set_ylabel('$C_Y$', fontsize=12)
    axes[0].set_title('Side Force Coefficient', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)

    # Cl vs beta
    axes[1].plot(beta_deg, Cl, 'r-', linewidth=2)
    axes[1].axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[1].axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[1].set_xlabel('Sideslip Angle (deg)', fontsize=12)
    axes[1].set_ylabel('$C_l$', fontsize=12)
    axes[1].set_title('Roll Moment Coefficient', fontsize=14, fontweight='bold')
    axes[1].grid(True, alpha=0.3)

    # Cn vs beta
    axes[2].plot(beta_deg, Cn, 'g-', linewidth=2)
    axes[2].axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[2].axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    axes[2].set_xlabel('Sideslip Angle (deg)', fontsize=12)
    axes[2].set_ylabel('$C_n$', fontsize=12)
    axes[2].set_title('Yaw Moment Coefficient', fontsize=14, fontweight='bold')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = output_dir / f'{model_name}_coeffs_vs_beta.png'
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")


def plot_polar(model, output_dir: Path, model_name: str):
    """Plot CL vs CD (drag polar)."""
    alpha_deg = np.linspace(-30, 30, 200)
    alpha_rad = np.deg2rad(alpha_deg)

    CL = model.CL(alpha_rad)
    CD = model.CD(alpha_rad)

    fig, ax = plt.subplots(figsize=(8, 6))

    # Color by alpha
    scatter = ax.scatter(CD, CL, c=alpha_deg, cmap='viridis', s=10, alpha=0.8)
    ax.plot(CD, CL, 'k-', linewidth=0.5, alpha=0.3)

    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Angle of Attack (deg)', fontsize=12)

    ax.axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    ax.axvline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.3)
    ax.set_xlabel('$C_D$', fontsize=12)
    ax.set_ylabel('$C_L$', fontsize=12)
    ax.set_title('Drag Polar', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = output_dir / f'{model_name}_drag_polar.png'
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")


def print_coefficients_table(model, output_dir: Path, model_name: str):
    """Print aerodynamic coefficients as a table."""
    alpha_deg = np.linspace(-30, 30, 200)
    alpha_rad = np.deg2rad(alpha_deg)

    CL = model.CL(alpha_rad)
    CD = model.CD(alpha_rad)
    Cm = model.Cm(alpha_rad)

    # Print table header
    print("\n" + "="*70)
    print(f"Aerodynamic Coefficients vs Angle of Attack: {model_name}")
    print("="*70)
    print(f"{'% Alpha':>12}  {'CL':>10}  {'CD':>10}  {'Cm':>10}")
    print("-"*70)

    # Print every 10th point to avoid too much output (20 points total)
    step = len(alpha_deg) // 20
    for i in range(0, len(alpha_deg), step):
        print(f"{alpha_deg[i]:12.4f}  {CL[i]:10.4f}  {CD[i]:10.4f}  {Cm[i]:10.4f}")

    print("="*70)

    # Also save to file
    table_path = output_dir / f'{model_name}_coefficients_table.txt'
    with open(table_path, 'w') as f:
        f.write(f"Aerodynamic Coefficients vs Angle of Attack: {model_name}\n")
        f.write(f"{'% Alpha':>12}  {'CL':>10}  {'CD':>10}  {'Cm':>10}\n")
        f.write("-"*70 + "\n")
        for i in range(len(alpha_deg)):
            f.write(f"{alpha_deg[i]:12.4f}  {CL[i]:10.4f}  {CD[i]:10.4f}  {Cm[i]:10.4f}\n")

    print(f"\nFull coefficient table saved to: {table_path}\n")


def print_coefficients_table_beta(model, output_dir: Path, model_name: str):
    """Print aerodynamic coefficients vs sideslip angle as a table (for AdvancedLiftDrag)."""
    if not isinstance(model, AdvancedLiftDragModel):
        return  # Only for advanced model

    beta_deg = np.linspace(-20, 20, 200)
    beta_rad = np.deg2rad(beta_deg)
    alpha_rad = np.zeros_like(beta_rad)  # Zero AoA

    CY = model.CY(alpha_rad, beta_rad)
    Cl = model.Cl(alpha_rad, beta_rad)
    Cn = model.Cn(alpha_rad, beta_rad)

    # Print table header
    print("\n" + "="*70)
    print(f"Aerodynamic Coefficients vs Sideslip Angle: {model_name}")
    print("="*70)
    print(f"{'% Beta':>12}  {'CY':>10}  {'Cl':>10}  {'Cn':>10}")
    print("-"*70)

    # Print every 10th point to avoid too much output (20 points total)
    step = len(beta_deg) // 20
    for i in range(0, len(beta_deg), step):
        print(f"{beta_deg[i]:12.4f}  {CY[i]:10.4f}  {Cl[i]:10.4f}  {Cn[i]:10.4f}")

    print("="*70)

    # Also save to file
    table_path = output_dir / f'{model_name}_coefficients_table_beta.txt'
    with open(table_path, 'w') as f:
        f.write(f"Aerodynamic Coefficients vs Sideslip Angle: {model_name}\n")
        f.write(f"{'% Beta':>12}  {'CY':>10}  {'Cl':>10}  {'Cn':>10}\n")
        f.write("-"*70 + "\n")
        for i in range(len(beta_deg)):
            f.write(f"{beta_deg[i]:12.4f}  {CY[i]:10.4f}  {Cl[i]:10.4f}  {Cn[i]:10.4f}\n")

    print(f"\nFull coefficient table (beta) saved to: {table_path}\n")


def print_model_summary(params: Dict):
    """Print a summary of model parameters."""
    print("\n" + "="*70)
    print(f"Model Type: {params['model_type']}")
    print("="*70)

    if params['model_type'] == 'LiftDrag':
        print(f"  Zero-alpha offset (a0):     {params['a0']:.6f} rad ({np.rad2deg(params['a0']):.2f}°)")
        print(f"  Lift slope (cla):            {params['cla']:.6f} /rad")
        print(f"  Drag slope (cda):            {params['cda']:.6f} /rad")
        print(f"  Moment slope (cma):          {params['cma']:.6f} /rad")
        print(f"  Stall angle:                 {np.rad2deg(params['alpha_stall']):.2f}°")
        print(f"  Post-stall lift slope:       {params['cla_stall']:.6f} /rad")
        print(f"  Post-stall drag slope:       {params['cda_stall']:.6f} /rad")

    elif params['model_type'] == 'AdvancedLiftDrag':
        print(f"  Zero-alpha offset (a0):      {params['a0']:.6f} rad ({np.rad2deg(params['a0']):.2f}°)")
        print(f"  Zero-alpha CL (CL0):         {params['CL0']:.6f}")
        print(f"  Zero-alpha CD (CD0):         {params['CD0']:.6f}")
        print(f"  Zero-alpha Cm (Cem0):        {params['Cem0']:.6f}")
        print(f"\n  Alpha derivatives:")
        print(f"    CLa:  {params['CLa']:8.4f} /rad")
        print(f"    Cema: {params['Cema']:8.4f} /rad")
        print(f"\n  Beta derivatives:")
        print(f"    CYb:   {params['CYb']:8.4f} /rad")
        print(f"    Cellb: {params['Cellb']:8.4f} /rad")
        print(f"    Cenb:  {params['Cenb']:8.4f} /rad")
        print(f"\n  Geometry:")
        print(f"    Aspect Ratio (AR):     {params['AR']:.2f}")
        print(f"    Oswald efficiency (e): {params['eff']:.3f}")
        print(f"    Wing area:             {params['area']:.3f} m²")
        print(f"    Stall angle:           {np.rad2deg(params['alpha_stall']):.2f}°")

    print("="*70 + "\n")


# ============================================================================
# Main
# ============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='Visualize aerodynamic model from Gazebo SDF file')
    parser.add_argument('sdf_file', type=Path,
                       help='Path to SDF model file')
    parser.add_argument('--output', type=Path, default=Path('plots/aero'),
                       help='Output directory for plots')
    parser.add_argument('--dpi', type=int, default=300,
                       help='Figure DPI')
    return parser.parse_args()


def main():
    args = parse_args()

    # Create output directory
    args.output.mkdir(parents=True, exist_ok=True)

    # Parse SDF
    print(f"Parsing SDF file: {args.sdf_file}")
    params = parse_sdf_file(args.sdf_file)

    # Print summary
    print_model_summary(params)

    # Create model
    if params['model_type'] == 'LiftDrag':
        model = LiftDragModel(params)
    else:
        model = AdvancedLiftDragModel(params)

    # Get model name from file
    model_name = args.sdf_file.parent.name

    # Print coefficient tables
    print_coefficients_table(model, args.output, model_name)
    if isinstance(model, AdvancedLiftDragModel):
        print_coefficients_table_beta(model, args.output, model_name)

    # Generate plots
    print(f"Generating plots...")
    plot_coefficients_vs_alpha(model, args.output, model_name)
    plot_polar(model, args.output, model_name)

    if isinstance(model, AdvancedLiftDragModel):
        plot_coefficients_vs_beta(model, args.output, model_name)

    print(f"\nAll plots saved to: {args.output}")


if __name__ == '__main__':
    main()
