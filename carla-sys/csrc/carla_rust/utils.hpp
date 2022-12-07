#pragma once

#include <cstddef>
#include <memory>
#include <vector>

namespace carla_rust
{
    namespace utils {
        template<typename T>
        class VectorRef {
        public:
            VectorRef(const std::vector<T> &ref)
                : inner_(ref)
            {}

            size_t len() const {
                return inner_.size();
            }

            const T* data() const {
                return inner_.data();
            }

        private:
            const std::vector<T> &inner_;
        };

        // Using the boost::shared_ptr to std::shared_ptr trick here:
        // https://stackoverflow.com/questions/71572186/question-on-converting-boost-shared-pointer-to-standard-shared-pointer
        // template<class T>
        // std::shared_ptr<T> as_std_shared_ptr(boost::shared_ptr<T> bp)
        // {
        //     if (!bp) return nullptr;
        //     auto pq = std::make_shared<boost::shared_ptr<T>>(std::move(bp));
        //     return std::shared_ptr<T>(pq, pq.get()->get());
        // }

        std::unique_ptr<std::vector<uint32_t>> new_vector_uint32_t() {
            return std::make_unique<std::vector<uint32_t>>();
        }
    }
}
