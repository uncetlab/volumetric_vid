#include <UVAtlas.h>
#include <DirectXMesh.h>
#include <memory>
#include <WaveFrontReader.h>
#include <stdio.h>
//#include <DirectXMath.h>

int main(int argc, char** argv)
{
	auto mesh = std::make_unique<WaveFrontReader<uint16_t>>();

	if (FAILED(mesh->Load(L"C:\\Users\\maxhu\\Desktop\\uvatlas_example\\teapot.obj"))) {
	//if (FAILED(mesh->Load(L"teapot.obj"))) {
		printf("Error loading .obj file");
		return 1;
	}

	size_t nFaces = mesh->indices.size() / 3;
	size_t nVerts = mesh->vertices.size();

	// copy vertex positions
	std::unique_ptr<DirectX::XMFLOAT3[]> pos(new DirectX::XMFLOAT3[nVerts]);
	for (size_t j = 0; j < nVerts; ++j)
		pos[j] = mesh->vertices[j].position;

	std::unique_ptr<uint32_t[]> adj(new uint32_t[mesh->indices.size()]);
	if (FAILED(DirectX::GenerateAdjacencyAndPointReps(&mesh->indices.front(), nFaces,
		pos.get(), nVerts, 0.f, nullptr, adj.get()))) {
		printf("ERROR");
		return 2;
	}

	std::vector<DirectX::UVAtlasVertex> vb;
	std::vector<uint8_t> ib;
	std::vector<uint32_t> remap;
	HRESULT hr = DirectX::UVAtlasCreate(pos.get(), nVerts,
		&mesh->indices.front(), DXGI_FORMAT_R16_UINT, nFaces,
		0, 0.f, 512, 512, 1.f,
		adj.get(), nullptr, nullptr,
		nullptr, DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
		DirectX::UVATLAS_DEFAULT, vb, ib,
		nullptr, &remap);
	if (FAILED(hr)) {
		printf("ERROR");
		return 3;
	}

	size_t nTotalVerts = vb.size();
	std::unique_ptr<WaveFrontReader<uint16_t>::Vertex[]> newVB(
		new WaveFrontReader<uint16_t>::Vertex[nTotalVerts]);

	hr = DirectX::UVAtlasApplyRemap(&mesh->vertices.front(),
		sizeof(WaveFrontReader<uint16_t>::Vertex),
		nVerts, nTotalVerts,
		&remap.front(), newVB.get());
	if (FAILED(hr)) {
		printf("ERROR");
		return 4;
	}

	 // Merge the UV data into the final VB
	for (size_t j = 0; j < nTotalVerts; ++j) {
		newVB[j].textureCoordinate = vb[j].uv;
		//(*newVB)[j].textureCoordinate = vb[j].uv;
	}
		
	auto newIB = reinterpret_cast<const uint16_t*>(&ib.front());
}
