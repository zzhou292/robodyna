#pragma once
namespace crash::modelio::vehicle {
// Ownership provenance, orthogonal to material law and native formulation.
// OriginalRigidPart reference values do not authorize independent shell motion.
enum class SourceShellRole { ConstitutiveShell, OriginalRigidPart };
}
